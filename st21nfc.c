/*
 * Copyright (C) 2016 ST Microelectronics S.A.
 * Copyright (C) 2010 Stollmann E+V GmbH
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 */
#define DEBUG

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include "st21nfc.h"
#include <linux/platform_data/st21nfc.h>
#include <linux/of_gpio.h>
#include <net/nfc/nci.h>
#include <linux/clk.h>

#define MAX_BUFFER_SIZE 260

#define DRIVER_VERSION "2.0.7"

#define PROP_PWR_MON_RW_ON_NTF nci_opcode_pack(NCI_GID_PROPRIETARY, 5)
#define PROP_PWR_MON_RW_OFF_NTF nci_opcode_pack(NCI_GID_PROPRIETARY, 6)

enum st21nfc_power_state {
	ST21NFC_IDLE,
	ST21NFC_ACTIVE,
	ST21NFC_ACTIVE_RW
};

enum st21nfc_read_state {
	ST21NFC_HEADER,
	ST21NFC_PAYLOAD
};

struct nfc_sub_power_stats {
	unsigned long count;
	unsigned long duration;
	unsigned long last_entry;
	unsigned long last_exit;
};

struct st21nfc_dev {
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice st21nfc_device;
	uint8_t buffer[MAX_BUFFER_SIZE];
	bool irq_enabled;
	struct st21nfc_platform_data platform_data;
	bool irqIsAttached;
	bool device_open; /* Is device open? */
	spinlock_t irq_enabled_lock;
	enum st21nfc_power_state pw_current;
	enum st21nfc_read_state r_state_current;
	int irq_pw_stats_idle;
	struct nfc_sub_power_stats pw_idle;
	struct nfc_sub_power_stats pw_active;
	struct nfc_sub_power_stats pw_active_rw;
	/* CLK control */
	bool			clk_run;
	struct	clk		*s_clk;
	int irqNumberClkReq;
};

/*
 * Routine to enable clock.
 * this routine can be extended to select from multiple
 * sources based on clk_src_name.
 */
static int st21nfc_clock_select(struct st21nfc_dev *st21nfc_dev)
{
	int ret = 0;

	st21nfc_dev->s_clk = clk_get(&st21nfc_dev->client->dev, "ref_clk");

	/* if NULL we assume external crystal and dont fail */
	if ((st21nfc_dev->s_clk == NULL) || IS_ERR(st21nfc_dev->s_clk))
		goto err_clk;

	if (st21nfc_dev->clk_run == false)
		ret = clk_prepare_enable(st21nfc_dev->s_clk);

	if (ret)
		goto err_clk;

	st21nfc_dev->clk_run = true;

	return ret;

err_clk:
	return -1;
}

/*
 * Routine to disable clocks
 */
static int st21nfc_clock_deselect(struct st21nfc_dev *st21nfc_dev)
{
	/* if NULL we assume external crystal and dont fail */
	if ((st21nfc_dev->s_clk == NULL) || IS_ERR(st21nfc_dev->s_clk))
		return -1;

	if (st21nfc_dev->clk_run == true) {
		clk_disable_unprepare(st21nfc_dev->s_clk);
		st21nfc_dev->clk_run = false;
	}
	return 0;
}

static irqreturn_t st21nfc_clkreq_irq_handler(int irq, void *dev_id)
{
	struct st21nfc_dev *st21nfc_dev = dev_id;
	int value = gpio_get_value(st21nfc_dev->platform_data.clkreq_gpio);
	int ret = 0;

	pr_debug("%s : enter\n", __func__);

	if (value != 0) {
		value = 1;
		ret = st21nfc_clock_select(st21nfc_dev);
		if (ret < 0)
			pr_err("%s : st21nfc_clock_select failed\n", __FILE__);
	} else {
		value = 0;
		ret = st21nfc_clock_deselect(st21nfc_dev);
		if (ret < 0)
			pr_err("%s : st21nfc_clock_deselect failed\n", __FILE__);
	}

	pr_debug("%s get gpio result %d\n", __func__, value);

	return IRQ_HANDLED;
}

static void st21nfc_disable_irq(struct st21nfc_dev *st21nfc_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&st21nfc_dev->irq_enabled_lock, flags);
	if (st21nfc_dev->irq_enabled) {
		pr_debug("%s : IRQ %d\n", __func__,
		 st21nfc_dev->client->irq);
		disable_irq_nosync(st21nfc_dev->client->irq);
		st21nfc_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&st21nfc_dev->irq_enabled_lock, flags);
}

static void st21nfc_enable_irq(struct st21nfc_dev *st21nfc_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&st21nfc_dev->irq_enabled_lock, flags);
	if (!st21nfc_dev->irq_enabled) {
		pr_debug("%s : IRQ %d\n", __func__,
		 st21nfc_dev->client->irq);
		st21nfc_dev->irq_enabled = true;
		enable_irq(st21nfc_dev->client->irq);

	}
	spin_unlock_irqrestore(&st21nfc_dev->irq_enabled_lock, flags);
}

static irqreturn_t st21nfc_dev_irq_handler(int irq, void *dev_id)
{
	struct st21nfc_dev *st21nfc_dev = dev_id;

	pr_debug("%s : enter\n", __func__);
	st21nfc_disable_irq(st21nfc_dev);

	/* Wake up waiting readers */
	wake_up(&st21nfc_dev->read_wq);

	return IRQ_HANDLED;
}

static int st21nfc_loc_set_polaritymode(struct st21nfc_dev *st21nfc_dev,
					int mode)
{
	struct i2c_client *client = st21nfc_dev->client;
	struct device *dev = &client->dev;
	unsigned int irq_type;
	int ret;

	st21nfc_dev->platform_data.polarity_mode = mode;
	/* setup irq_flags */
	switch (mode) {
	case IRQF_TRIGGER_RISING:
		irq_type = IRQ_TYPE_EDGE_RISING;
		break;
	case IRQF_TRIGGER_HIGH:
		irq_type = IRQ_TYPE_LEVEL_HIGH;
		break;
	default:
		irq_type = IRQ_TYPE_EDGE_RISING;
		break;
	}
	if (st21nfc_dev->irqIsAttached) {
		devm_free_irq(dev, client->irq, st21nfc_dev);
		st21nfc_dev->irqIsAttached = false;
	}
	ret = irq_set_irq_type(client->irq, irq_type);
	if (ret) {
		pr_err("%s : set_irq_type failed\n", __FILE__);
		return -ENODEV;
	}
	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	pr_debug("%s : requesting IRQ %d\n", __func__, client->irq);
	st21nfc_dev->irq_enabled = true;

	ret = devm_request_irq(dev, client->irq, st21nfc_dev_irq_handler,
				st21nfc_dev->platform_data.polarity_mode,
				client->name, st21nfc_dev);
	if (ret) {
		pr_err("%s : devm_request_irq failed\n", __FILE__);
		return -ENODEV;
	}
	st21nfc_dev->irqIsAttached = true;
	st21nfc_disable_irq(st21nfc_dev);

	return ret;
}

static void st21nfc_power_stats_idle_signal(struct st21nfc_dev *st21nfc_dev)
{
	unsigned long current_time_ms = ktime_to_ms(ktime_get_boottime());
	int value = gpio_get_value(st21nfc_dev->platform_data.pidle_gpio);

	if (value != 0) {
		switch(st21nfc_dev->pw_current) {
		case ST21NFC_IDLE:
			pr_info("%s Switching from Idle mode to Active mode: %lx \n", __func__, current_time_ms);
			st21nfc_dev->pw_active.last_entry = current_time_ms;
			st21nfc_dev->pw_idle.last_exit = current_time_ms;
			st21nfc_dev->pw_idle.duration += (st21nfc_dev->pw_idle.last_exit -
											  st21nfc_dev->pw_idle.last_entry);
			st21nfc_dev->pw_active.count++;
			st21nfc_dev->pw_current = ST21NFC_ACTIVE;
			break;
		default:
			pr_err("%s Error: Switch to Active mode when previously not in IDLE mode!: %lx\n", __func__, current_time_ms);
			return;
		}
	} else {
		switch (st21nfc_dev->pw_current) {
		case ST21NFC_IDLE:
			if (st21nfc_dev->pw_idle.last_entry != 0) {
				pr_err("%s Error: Switched from idle mode to idle mode!: %lx\n", __func__, current_time_ms);
				return;
			}
			break;
		case ST21NFC_ACTIVE:
			pr_info("%s Switching from Active mode to Idle mode: %lx \n", __func__, current_time_ms);
			st21nfc_dev->pw_active.last_exit = current_time_ms;
			st21nfc_dev->pw_active.duration += (st21nfc_dev->pw_active.last_exit -
												st21nfc_dev->pw_active.last_entry);
			break;
		case ST21NFC_ACTIVE_RW:
			pr_info("%s Switching from Active RW mode to Idle mode: %lx \n", __func__, current_time_ms);
			st21nfc_dev->pw_active_rw.last_exit = current_time_ms;
			st21nfc_dev->pw_active.duration += (st21nfc_dev->pw_active_rw.last_exit -
												st21nfc_dev->pw_active_rw.last_entry);
			break;
		}

		st21nfc_dev->pw_idle.count++;
		st21nfc_dev->pw_current = ST21NFC_IDLE;
		st21nfc_dev->pw_idle.last_entry = current_time_ms;
	}
}

static irqreturn_t st21nfc_dev_power_stats_handler(int irq, void *dev_id)
{
	struct st21nfc_dev *st21nfc_dev = dev_id;

	pr_debug("%s : enter\n", __func__);

	st21nfc_power_stats_idle_signal(st21nfc_dev);

	return IRQ_HANDLED;
}

static void st21nfc_power_stats_filter(struct st21nfc_dev *st21nfc_dev, char *buf, size_t count)
{
	unsigned long current_time_ms = ktime_to_ms(ktime_get_boottime());
	__u16 ntf_opcode = nci_opcode(buf);

	/* In order to avoid counting active state on PAYLOAD where it would match
	 * a possible header, power states are filtered only on NCI headers.
	 */
	if (st21nfc_dev->r_state_current != ST21NFC_HEADER)
		goto exit;

	if (nci_mt(buf) != NCI_MT_NTF_PKT && nci_opcode_gid(ntf_opcode) != NCI_GID_PROPRIETARY)
		goto exit;

	switch (ntf_opcode) {
	case PROP_PWR_MON_RW_OFF_NTF:
		switch (st21nfc_dev->pw_current) {
		case ST21NFC_ACTIVE:
			pr_err("%s Error: Switched from Active mode to Active mode!: %lx\n", __func__, current_time_ms);
			return;
		case ST21NFC_IDLE:
			pr_info("%s Switching from Idle mode to Active mode: %lx \n", __func__, current_time_ms);
			st21nfc_dev->pw_idle.last_exit = current_time_ms;
			st21nfc_dev->pw_idle.duration +=
				(st21nfc_dev->pw_idle.last_exit - st21nfc_dev->pw_idle.last_entry);
		break;
		case ST21NFC_ACTIVE_RW:
			pr_info("%s Switching from Active RW mode to Active mode: %lx \n", __func__, current_time_ms);
			st21nfc_dev->pw_active_rw.last_exit = current_time_ms;
			st21nfc_dev->pw_active_rw.duration +=
				(st21nfc_dev->pw_active_rw.last_exit - st21nfc_dev->pw_active_rw.last_entry);
		break;
		}

		st21nfc_dev->pw_current = ST21NFC_ACTIVE;
		st21nfc_dev->pw_active.count++;
		st21nfc_dev->pw_active.last_entry = current_time_ms;
		break;
	case PROP_PWR_MON_RW_ON_NTF:
		switch (st21nfc_dev->pw_current) {
		case ST21NFC_ACTIVE_RW:
			pr_err("Error: Switched from Active mode to Active mode!\n");
			return;
		case ST21NFC_IDLE:
			pr_info("%s Switching from Idle mode to Active RW mode: %lx \n", __func__, current_time_ms);
			st21nfc_dev->pw_idle.last_exit = current_time_ms;
			st21nfc_dev->pw_idle.duration +=
				(st21nfc_dev->pw_idle.last_exit - st21nfc_dev->pw_idle.last_entry);
			break;
		case ST21NFC_ACTIVE:
			pr_info("%s Switching from Active mode to Active RW mode: %lx \n", __func__, current_time_ms);
			st21nfc_dev->pw_active.last_exit = current_time_ms;
			st21nfc_dev->pw_active.duration +=
				(st21nfc_dev->pw_active.last_exit - st21nfc_dev->pw_active.last_entry);
			break;
		}
		st21nfc_dev->pw_current = ST21NFC_ACTIVE_RW;
		st21nfc_dev->pw_active_rw.count++;
		st21nfc_dev->pw_active_rw.last_entry = current_time_ms;
		break;
	default:
		goto exit;
	}

	return;
exit:
	if (st21nfc_dev->r_state_current == ST21NFC_HEADER) {
		st21nfc_dev->r_state_current = ST21NFC_PAYLOAD;
	} else {
		st21nfc_dev->r_state_current = ST21NFC_HEADER;
	}
}

static ssize_t st21nfc_dev_read(struct file *filp, char __user *buf,
				size_t count, loff_t *offset)
{
	struct st21nfc_dev *st21nfc_dev = container_of(filp->private_data,
						       struct st21nfc_dev,
						       st21nfc_device);
	int ret;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	pr_debug("%s : reading %zu bytes.\n", __func__, count);

	mutex_lock(&st21nfc_dev->read_mutex);

	/* Read data */
	ret = i2c_master_recv(st21nfc_dev->client, st21nfc_dev->buffer, count);
	mutex_unlock(&st21nfc_dev->read_mutex);

	if (ret < 0) {
		pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) {
		pr_err("%s: received too many bytes from i2c (%d)\n",
		       __func__, ret);
		return -EIO;
	}


	st21nfc_power_stats_filter(st21nfc_dev, st21nfc_dev->buffer, ret);

	if (copy_to_user(buf, st21nfc_dev->buffer, ret)) {
		pr_warn("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	return ret;
}

static ssize_t st21nfc_dev_write(struct file *filp, const char __user *buf,
				 size_t count, loff_t *offset)
{
	struct st21nfc_dev *st21nfc_dev = container_of(filp->private_data,
				   struct st21nfc_dev, st21nfc_device);
	char *tmp = NULL;
	int ret = count;

	pr_debug("%s: st21nfc_dev ptr %p\n", __func__, st21nfc_dev);

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	tmp = memdup_user(buf, count);
	if (IS_ERR(tmp)) {
		pr_err("%s : memdup_user failed\n", __func__);
		return -EFAULT;
	}

	pr_debug("%s : writing %zu bytes.\n", __func__, count);
	/* Write data */
	ret = i2c_master_send(st21nfc_dev->client, tmp, count);
	if (ret != count) {
		pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}
	kfree(tmp);

	return ret;
}

static int st21nfc_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct st21nfc_dev *st21nfc_dev = container_of(filp->private_data,
						       struct st21nfc_dev,
						       st21nfc_device);

	if (st21nfc_dev->device_open) {
		ret = -EBUSY;
		pr_err("%s : device already opened ret= %d\n", __func__, ret);
	} else {
		st21nfc_dev->device_open = true;

		pr_debug("%s : device_open = %d", __func__, st21nfc_dev->device_open);
		pr_debug("%s : %d,%d ", __func__, imajor(inode), iminor(inode));

		pr_debug("%s: st21nfc_dev ptr %p\n", __func__, st21nfc_dev);
	}
	return ret;
}


static int st21nfc_release(struct inode *inode, struct file *file)
{
	struct st21nfc_dev *st21nfc_dev = container_of(file->private_data,
						       struct st21nfc_dev,
						       st21nfc_device);

	st21nfc_dev->device_open = false;
	pr_debug("%s : device_open  = %d\n", __func__, st21nfc_dev->device_open);

	return 0;
}

static long st21nfc_dev_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	struct st21nfc_dev *st21nfc_dev = container_of(filp->private_data,
							struct st21nfc_dev,
							st21nfc_device);
	struct st21nfc_platform_data *pdata = &st21nfc_dev->platform_data;
	int ret = 0;

	pr_info("%s cmd=%d", __func__, cmd);

	switch (cmd) {

	case ST21NFC_SET_POLARITY_RISING:
		pr_info(" ### ST21NFC_SET_POLARITY_RISING ###");
		st21nfc_loc_set_polaritymode(st21nfc_dev, IRQF_TRIGGER_RISING);
		break;

	case ST21NFC_SET_POLARITY_HIGH:
		pr_info(" ### ST21NFC_SET_POLARITY_HIGH ###");
		st21nfc_loc_set_polaritymode(st21nfc_dev, IRQF_TRIGGER_HIGH);
		break;

	case ST21NFC_PULSE_RESET:
		/* Double pulse is done to exit Quick boot mode.*/
		pr_info("%s Double Pulse Request\n", __func__);
		if (gpio_is_valid(pdata->reset_gpio)) {
			/* pulse low for 20 millisecs */
			gpio_set_value(pdata->reset_gpio, 0);
			msleep(20);
			gpio_set_value(pdata->reset_gpio, 1);
			usleep_range(10000, 10100);
			/* pulse low for 20 millisecs */
			gpio_set_value(pdata->reset_gpio, 0);
			msleep(20);
			gpio_set_value(pdata->reset_gpio, 1);
			pr_info("%s done Double Pulse Request\n", __func__);
		}
		break;

	case ST21NFC_GET_WAKEUP:
		/* deliver state of Wake_up_pin as return value of ioctl */
		ret = gpio_get_value(pdata->irq_gpio);
		/*
		 * Warning: depending on gpio_get_value implementation,
		 * it can returns a value different than 1 in case of high level
		 */
		if (ret != 0)
			ret = 1;
		else
			ret = 0;
		pr_debug("%s get gpio result %d\n", __func__, ret);
		break;
	case ST21NFC_GET_POLARITY:
		ret = pdata->polarity_mode;
		pr_debug("%s get polarity %d\n", __func__, ret);
		break;
	case ST21NFC_RECOVERY:
		/* For ST21NFCD usage only */
		pr_info("%s Recovery Request\n", __func__);
		if (gpio_is_valid(pdata->reset_gpio)) {
			/* pulse low for 20 millisecs */
			gpio_set_value(pdata->reset_gpio, 0);
			usleep_range(10000, 10100);
			/* during the reset, force IRQ OUT as */
			/* DH output instead of input in normal usage */
			ret = gpio_direction_output(pdata->irq_gpio, 1);
			if (ret) {
				pr_err("%s : gpio_direction_output failed\n",
				 __FILE__);
				ret = -ENODEV;
				break;
			}
			gpio_set_value(pdata->irq_gpio, 1);
			usleep_range(10000, 10100);
			gpio_set_value(pdata->reset_gpio, 1);
			pr_info("%s done Pulse Request\n", __func__);
		}
		msleep(20);
		gpio_set_value(pdata->irq_gpio, 0);
		msleep(20);
		gpio_set_value(pdata->irq_gpio, 1);
		msleep(20);
		gpio_set_value(pdata->irq_gpio, 0);
		msleep(20);
		pr_info("%s Recovery procedure finished\n", __func__);
		ret = gpio_direction_input(pdata->irq_gpio);
		if (ret) {
			pr_err("%s : gpio_direction_input failed\n", __FILE__);
			ret = -ENODEV;
		}
		break;
	default:
		pr_err("%s bad ioctl %u\n", __func__, cmd);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static unsigned int st21nfc_poll(struct file *file, poll_table *wait)
{
	struct st21nfc_dev *st21nfc_dev = container_of(file->private_data,
						       struct st21nfc_dev,
						       st21nfc_device);
	unsigned int mask = 0;
	int pinlev = 0;

	/* wait for Wake_up_pin == high  */
	poll_wait(file, &st21nfc_dev->read_wq, wait);

	pinlev = gpio_get_value(st21nfc_dev->platform_data.irq_gpio);

	if (pinlev != 0) {
		pr_debug("%s return ready\n", __func__);
		mask = POLLIN | POLLRDNORM;	/* signal data avail */
		st21nfc_disable_irq(st21nfc_dev);
	} else {
		/* Wake_up_pin is low. Activate ISR  */
		if (!st21nfc_dev->irq_enabled) {
			pr_debug("%s enable irq\n", __func__);
			st21nfc_enable_irq(st21nfc_dev);
		} else {
			pr_debug("%s irq already enabled\n", __func__);
		}
	}
	return mask;
}

static const struct file_operations st21nfc_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read = st21nfc_dev_read,
	.write = st21nfc_dev_write,
	.open = st21nfc_dev_open,
	.poll = st21nfc_poll,
	.release = st21nfc_release,

	.unlocked_ioctl = st21nfc_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = st21nfc_dev_ioctl
#endif
};

static ssize_t st21nfc_show_i2c_addr(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);

	if (client != NULL)
		return sprintf(buf, "0x%.2x\n", client->addr);
	return 0;
}				/* st21nfc_show_i2c_addr() */

static ssize_t st21nfc_change_i2c_addr(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct st21nfc_dev *data = dev_get_drvdata(dev);
	long new_addr = 0;

	if (data != NULL && data->client != NULL) {
		if (!kstrtol(buf, 10, &new_addr)) {
			mutex_lock(&data->read_mutex);
			data->client->addr = new_addr;
			mutex_unlock(&data->read_mutex);
			return count;
		}
		return -EINVAL;
	}
	return 0;
}				/* st21nfc_change_i2c_addr() */

static ssize_t st21nfc_version(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", DRIVER_VERSION);
}				/* st21nfc_version */

static ssize_t st21nfc_show_power_stats(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct st21nfc_dev *data = dev_get_drvdata(dev);
	unsigned long current_time_ms = ktime_to_ms(ktime_get_boottime());
	unsigned long idle_duration = data->pw_current != ST21NFC_IDLE ?
								data->pw_idle.duration :
								data->pw_idle.duration + (current_time_ms - data->pw_idle.last_entry);
	unsigned long active_ce_duration = data->pw_current != ST21NFC_ACTIVE ?
								data->pw_active.duration :
								data->pw_active.duration + (current_time_ms - data->pw_active.last_entry);
	unsigned long active_rw_duration = data->pw_current != ST21NFC_ACTIVE_RW ?
								data->pw_active_rw.duration :
								data->pw_active_rw.duration + (current_time_ms - data->pw_active_rw.last_entry);

	return sprintf(buf, "NFC subsystem\n"
			    "Idle mode:\n"
			    "\tCumulative count: 0x%lx\n"
			    "\tCumulative duration msec: 0x%lx\n"
			    "\tLast entry timestamp msec: 0x%lx\n"
			    "\tLast exit timestamp msec: 0x%lx\n"
			    "Active mode:\n"
			    "\tCumulative count: 0x%lx\n"
			    "\tCumulative duration msec: 0x%lx\n"
			    "\tLast entry timestamp msec: 0x%lx\n"
			    "\tLast exit timestamp msec: 0x%lx\n"
			    "Active Reader/Writer mode:\n"
			    "\tCumulative count: 0x%lx\n"
			    "\tCumulative duration msec: 0x%lx\n"
			    "\tLast entry timestamp msec: 0x%lx\n"
			    "\tLast exit timestamp msec: 0x%lx\n"
			    "\n\tTotal uptime: 0x%lx Cumulative modes time: 0x%lx\n",
			    data->pw_idle.count,
			    idle_duration,
			    data->pw_idle.last_entry,
			    data->pw_idle.last_exit,
			    data->pw_active.count,
			    active_ce_duration,
			    data->pw_active.last_entry,
			    data->pw_active.last_exit,
			    data->pw_active_rw.count,
			    active_rw_duration,
			    data->pw_active_rw.last_entry,
			    data->pw_active_rw.last_exit,
			    current_time_ms,
			    idle_duration + active_ce_duration + active_rw_duration);
}

static DEVICE_ATTR(i2c_addr, S_IRUGO | S_IWUSR, st21nfc_show_i2c_addr,
		   st21nfc_change_i2c_addr);

static DEVICE_ATTR(version, S_IRUGO, st21nfc_version, NULL);

static DEVICE_ATTR(power_stats, S_IRUGO, st21nfc_show_power_stats, NULL);

static struct attribute *st21nfc_attrs[] = {
	&dev_attr_i2c_addr.attr,
	&dev_attr_version.attr,
	&dev_attr_power_stats.attr,
	NULL,
};

static struct attribute_group st21nfc_attr_grp = {
	.attrs = st21nfc_attrs,
};

#ifdef CONFIG_OF
static int st21nfc_parse_dt(struct device *dev, struct st21nfc_platform_data *pdata)
{
	int ret = 0;
	struct device_node *np = dev->of_node;

	np = of_find_compatible_node(NULL, NULL, "st,st21nfc");
	if (IS_ERR_OR_NULL(np)) {
		pr_err("[dsc]%s: cannot find compatible node \"%s\"",
		 __func__, "st,st21nfc");
		return -ENODEV;
	}

	pdata->reset_gpio = of_get_named_gpio(np, "st,reset_gpio", 0);
	if ((!gpio_is_valid(pdata->reset_gpio))) {
		pr_err("[dsc]%s: fail to get reset_gpio\n", __func__);
		return -EINVAL;
	}
	pdata->irq_gpio = of_get_named_gpio(np, "st,irq_gpio", 0);
	if ((!gpio_is_valid(pdata->irq_gpio))) {
		pr_err("[dsc]%s: fail to get irq_gpio\n", __func__);
		return -EINVAL;
	}

	pdata->pidle_gpio = of_get_named_gpio(np, "st,pidle_gpio", 0);
	if ((!gpio_is_valid(pdata->pidle_gpio))) {
		pr_err("[dsc]%s: [OPTIONAL] fail to get pidle_gpio\n", __func__);
	}

	pdata->clkreq_gpio = of_get_named_gpio(np, "st,clkreq_gpio", 0);
	if ((!gpio_is_valid(pdata->clkreq_gpio)))
		pr_err("[dsc]%s: [OPTIONAL] fail to get clkreq_gpio\n",
		 __func__);

	pdata->polarity_mode = IRQF_TRIGGER_RISING;
	pr_err("[dsc]%s : get reset_gpio[%d], \
	 irq_gpio[%d], pidle_gpio[%d], clkreq_gpio[%d], polarity_mode[%d]\n",
	  __func__, pdata->reset_gpio,
	   pdata->irq_gpio, pdata->pidle_gpio,
	   pdata->clkreq_gpio, pdata->polarity_mode);
	return ret;
}
#else
static int st21nfc_parse_dt(struct device *dev, struct st21nfc_platform_data *pdata)
{
	return 0;
}
#endif


static int st21nfc_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	int ret;
	struct st21nfc_platform_data *pdata;
	struct st21nfc_dev *st21nfc_dev;
	struct device *dev = &client->dev;

	pr_info("st21nfc_probe\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s : need I2C_FUNC_I2C\n", __func__);
		return -ENODEV;
	}

	if (dev->of_node) {
		pdata = devm_kzalloc(dev,
			sizeof(struct st21nfc_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(dev, "nfc-nci prob: Failed to allocate memory\n");
			return -ENOMEM;
		}
		pr_info("%s : Parse st21nfc DTS\n", __func__);
		ret = st21nfc_parse_dt(dev, pdata);
		if (ret)
			return ret;
	} else {
		pdata = dev->platform_data;
		pr_info("%s : No st21nfc DTS\n", __func__);
	}

	if (!pdata) {
		dev_err(dev, "nfc-nci probe: failed\n");
		return -ENODEV;
	}

	dev_dbg(dev, "nfc-nci probe: %s, inside nfc-nci flags = %x\n",
		__func__, client->flags);

	client->adapter->timeout = msecs_to_jiffies(3 * 10);	/* 30ms */
	client->adapter->retries = 0;

	st21nfc_dev = devm_kzalloc(dev, sizeof(*st21nfc_dev), GFP_KERNEL);
	if (st21nfc_dev == NULL) {
		dev_err(dev, "failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	pr_debug("%s : dev_cb_addr %p\n", __func__, st21nfc_dev);

	/* store for later use */
	st21nfc_dev->platform_data.irq_gpio = pdata->irq_gpio;
	st21nfc_dev->platform_data.clkreq_gpio = pdata->clkreq_gpio;
	st21nfc_dev->platform_data.pidle_gpio = pdata->pidle_gpio;
	st21nfc_dev->platform_data.reset_gpio = pdata->reset_gpio;
	st21nfc_dev->platform_data.polarity_mode = pdata->polarity_mode;
	st21nfc_dev->client = client;

	ret = devm_gpio_request_one(dev, pdata->irq_gpio, GPIOF_IN, "irq_gpio");
	if (ret) {
		pr_err("%s : devm_gpio_request_one failed\n", __FILE__);
		return ret;
	}

	ret = devm_gpio_request_one(dev, pdata->reset_gpio, GPIOF_OUT_INIT_HIGH, "reset_gpio");
	if (ret) {
		pr_err("%s : reset devm_gpio_request_one failed\n", __FILE__);
		return ret;
	}

	if (gpio_is_valid(pdata->pidle_gpio)) {
		ret = devm_gpio_request_one(dev, pdata->pidle_gpio, GPIOF_IN, "pidle_gpio");
		if (ret) {
			pr_err("%s : pidle devm_gpio_request_one failed\n", __FILE__);
			return ret;
		}

		/* Start the power stat in power mode idle */
		st21nfc_dev->r_state_current = ST21NFC_HEADER;
		st21nfc_dev->irq_pw_stats_idle = gpio_to_irq(pdata->pidle_gpio);

		ret = irq_set_irq_type(st21nfc_dev->irq_pw_stats_idle, IRQ_TYPE_EDGE_BOTH);
		if (ret) {
			pr_err("%s : set_irq_type failed\n", __FILE__);
			return ret;
		}

		/* This next call requests an interrupt line */
		ret = devm_request_irq(dev, st21nfc_dev->irq_pw_stats_idle,
							(irq_handler_t) st21nfc_dev_power_stats_handler,
							IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, /* Interrupt on both edges */
							"st21nfc_pw_stats_idle_handle",
							st21nfc_dev);
		if (ret) {
			pr_err("%s : devm_request_irq for power stats idle failed\n", __FILE__);
			return ret;
		}
	}

	if (gpio_is_valid(pdata->clkreq_gpio)) {
		ret = devm_gpio_request_one(dev, pdata->clkreq_gpio, GPIOF_IN, "clkreq_gpio");
		if (ret) {
			pr_err("%s : [OPTIONAL] gpio_request failed\n", __FILE__);
			return ret;
		}

		/* handle clk_req irq */
		st21nfc_dev->irqNumberClkReq = gpio_to_irq(pdata->clkreq_gpio);

		ret = irq_set_irq_type(st21nfc_dev->irqNumberClkReq, IRQ_TYPE_EDGE_BOTH);
		if (ret) {
			pr_err("%s : irq_set_irq_type for clkreq failed\n", __FILE__);
			return ret;
		}

		/* This next call requests an interrupt line */
		ret = devm_request_irq(dev, st21nfc_dev->irqNumberClkReq,
							(irq_handler_t) st21nfc_clkreq_irq_handler,
							IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, /* Interrupt on both edges */
							"st21nfc_clkreq_handler",
							st21nfc_dev);
		if (ret) {
			pr_err("%s : devm_request_irq for clkreq failed\n", __FILE__);
			return ret;
		}

		ret = st21nfc_clock_select(st21nfc_dev);
		if (ret < 0) {
			pr_err("%s : st21nfc_clock_select failed\n", __FILE__);
			return ret;
		}
	}

	client->irq = gpio_to_irq(pdata->irq_gpio);

	/* init mutex and queues */
	init_waitqueue_head(&st21nfc_dev->read_wq);
	mutex_init(&st21nfc_dev->read_mutex);
	spin_lock_init(&st21nfc_dev->irq_enabled_lock);
	pr_debug("%s : debug irq_gpio = %d, client-irq =  %d\n",
			__func__, pdata->irq_gpio, client->irq);
	st21nfc_dev->st21nfc_device.minor = MISC_DYNAMIC_MINOR;
	st21nfc_dev->st21nfc_device.name = "st21nfc";
	st21nfc_dev->st21nfc_device.fops = &st21nfc_dev_fops;
	st21nfc_dev->st21nfc_device.parent = dev;

	i2c_set_clientdata(client, st21nfc_dev);
	ret = misc_register(&st21nfc_dev->st21nfc_device);
	if (ret) {
		pr_err("%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	ret = sysfs_create_group(&dev->kobj, &st21nfc_attr_grp);
	if (ret) {
		pr_err("%s : sysfs_create_group failed\n", __FILE__);
		goto err_sysfs_create_group_failed;
	}

	return 0;

err_sysfs_create_group_failed:
	misc_deregister(&st21nfc_dev->st21nfc_device);
err_misc_register:
	mutex_destroy(&st21nfc_dev->read_mutex);
	return ret;
}

static int st21nfc_remove(struct i2c_client *client)
{
	struct st21nfc_dev *st21nfc_dev = i2c_get_clientdata(client);

	st21nfc_clock_deselect(st21nfc_dev);
	misc_deregister(&st21nfc_dev->st21nfc_device);
	sysfs_remove_group(&client->dev.kobj, &st21nfc_attr_grp);
	mutex_destroy(&st21nfc_dev->read_mutex);

	return 0;
}

static const struct i2c_device_id st21nfc_id[] = {
	{"st21nfc", 0},
	{}
};

static const struct of_device_id st21nfc_of_match[] = {
	{ .compatible = "st,st21nfc", },
	{}
};

/* MODULE_DEVICE_TABLE(of, st21nfc_of_match); */
static struct i2c_driver st21nfc_driver = {
	.id_table = st21nfc_id,
	.driver = {
		.name	= "st21nfc",
		.owner	= THIS_MODULE,
		.of_match_table	= st21nfc_of_match,
	},
	.probe		= st21nfc_probe,
	.remove		= st21nfc_remove,
};

#if 0
static const struct of_device_id st21nfc_board_of_match[] = {
	{ .compatible = "st,st21nfc", },
	{}
};

static int st21nfc_platform_probe(struct platform_device *pdev)
{
	pr_info("st21nfc_platform_probe pr_info\n");
	pr_err("st21nfc_platform_probe pr_err\n");
	return 0;
}

static int st21nfc_platform_remove(struct platform_device *pdev)
{
	pr_err("st21nfc_platform_remove\n");
	return 0;
}


static struct platform_driver st21nfc_platform_driver = {
	.probe = st21nfc_platform_probe,
	.remove = st21nfc_platform_remove,
	.driver = {
		.name	= "st21nfc",
		.owner	= THIS_MODULE,
		.of_match_table	= st21nfc_board_of_match,
	},
};
#endif
/*
 * module load/unload record keeping
 */

static int __init st21nfc_dev_init(void)
{
	pr_info("%s: Loading st21nfc driver\n", __func__);
#if 0
	/* add by wuling to fix compilation error */
	platform_driver_register(&st21nfc_platform_driver);
#endif
	return i2c_add_driver(&st21nfc_driver);
}

module_init(st21nfc_dev_init);

static void __exit st21nfc_dev_exit(void)
{
	pr_debug("Unloading st21nfc driver\n");
	i2c_del_driver(&st21nfc_driver);
}

module_exit(st21nfc_dev_exit);

MODULE_AUTHOR("STMicroelectronics");
MODULE_DESCRIPTION("NFC ST21NFC driver");
MODULE_VERSION(DRIVER_VERSION);
MODULE_LICENSE("GPL");
