/* drivers/nfc/ese/st54j_se.c
 * Copyright (C) 2016 ST Microelectronics S.A.
 * Copyright 2017 Google Inc.
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/spi/spi.h>
#include <linux/uaccess.h>


#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/version.h>

#include <linux/semaphore.h>
#include <linux/completion.h>

#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/nfcinfo.h>

#ifdef CONFIG_COMPAT
#include <linux/compat.h>
#endif

#include "st54j_se.h"
#include "../st21nfc.h"

#define DRIVER_VERSION "1.1.1"
#define ST54_MAX_BUF 258U

struct ese_dev {
	struct spi_device	*spi;
	struct device		*nfcc_device;
	struct st21nfc_dev	*nfcc_data;
	struct mutex		mutex;
	struct	miscdevice	device;
	const char		*nfcc_name;
	unsigned int ese_reset_gpio;
};

#ifdef CONFIG_COMPAT
static long ese_compat_ioctl(struct file *filp, unsigned int cmd,
				unsigned long arg)
{
	long r = 0;
	struct ese_dev *ese_dev = filp->private_data;
	mutex_lock(&ese_dev->mutex);

	arg = (compat_u64)arg;

	mutex_unlock(&ese_dev->mutex);
	return r;
}
#endif

static long ese_ioctl(struct file *filp, unsigned int cmd,
			unsigned long arg)
{
	int r = 0;
	struct ese_dev *ese_dev = filp->private_data;
	mutex_lock(&ese_dev->mutex);
	dev_info(&ese_dev->spi->dev,
			"%s: enter, cmd=%d\n", __func__, cmd);
			
	switch (cmd) {
		case ST54J_SE_RESET:
			pr_info("%s  Reset Request received\n", __func__);
			if (ese_dev->ese_reset_gpio != 0) {
				/* if GPIO was equal to 0 (following power ON), first ensure it is high during 20ms */
				if (gpio_get_value(ese_dev->ese_reset_gpio) == 0)
				{
					pr_info("%s detects ese_reset_gpio to low, first force to high for 20ms\n", __func__);
					gpio_set_value(ese_dev->ese_reset_gpio, 1);
					msleep(20);
				}
				/* pulse low for 20 millisecs */
				gpio_set_value(ese_dev->ese_reset_gpio, 0);
				msleep(20);
				gpio_set_value(ese_dev->ese_reset_gpio, 1);
				pr_info("%s sent Reset request on eSE\n", __func__);
			}
		break;
	}
			
	mutex_unlock(&ese_dev->mutex);
	return r;
}

static int ese_open(struct inode *inode, struct file *filp)
{
	struct ese_dev *ese_dev = container_of(filp->private_data,
				struct ese_dev, device);
				
	dev_info(&ese_dev->spi->dev,
			"%s: enter\n", __func__);
			pr_err("%s : open st54j_se \n", __func__);
	mutex_lock(&ese_dev->mutex);
	/* Find the NFC parent device if it exists. */
	if (ese_dev->nfcc_data == NULL) {
		struct device *nfc_dev = bus_find_device_by_name(
					&i2c_bus_type,
					NULL,
					ese_dev->nfcc_name);
		if (!nfc_dev) {
			dev_err(&ese_dev->spi->dev,
				"%s: cannot find NFC controller '%s'\n",
				__func__, ese_dev->nfcc_name);
			goto err;
		}
		ese_dev->nfcc_data = dev_get_drvdata(nfc_dev);
		if (!ese_dev->nfcc_data) {
			dev_err(&ese_dev->spi->dev,
				"%s: cannot find NFC controller device data\n",
				__func__);
			put_device(nfc_dev);
			goto err;
		}
		dev_dbg(&ese_dev->spi->dev,
				"%s: NFC controller found\n", __func__);
		ese_dev->nfcc_device = nfc_dev;
	}
	mutex_unlock(&ese_dev->mutex);

	filp->private_data = ese_dev;
	dev_dbg(&ese_dev->spi->dev,
			"%s: opened st54j_se\n", __func__);


	return 0;

err:
	mutex_unlock(&ese_dev->mutex);
	return -ENODEV;
}

static int ese_release(struct inode *ino, struct file *filp)
{
	struct ese_dev *ese_dev = filp->private_data;
	int pwr = 0;

	//pwr = ese_ioctl(filp, ESE_GET_PWR, 0);

	mutex_lock(&ese_dev->mutex);
	dev_dbg(&ese_dev->spi->dev,
			"%s: power: %d\n", __func__, pwr);
	mutex_unlock(&ese_dev->mutex);
	return 0;
}

static ssize_t ese_write(struct file *filp, const char __user *ubuf,
				size_t len, loff_t *offset)
{
	struct ese_dev *ese_dev = filp->private_data;
	int ret = -EFAULT;
	size_t bytes = len;
	char tx_buf[ST54_MAX_BUF];

	if (len > INT_MAX)
		return -EINVAL;

	mutex_lock(&ese_dev->mutex);
	while (bytes > 0) {
		size_t block = bytes < sizeof(tx_buf) ? bytes : sizeof(tx_buf);

		memset(tx_buf, 0, sizeof(tx_buf));
		if (copy_from_user(tx_buf, ubuf, block)) {
			dev_dbg(&ese_dev->spi->dev, "failed to copy from user\n");
			goto err;
		}

		ret = spi_write(ese_dev->spi, tx_buf, block);

		if (ret < 0) {
			dev_dbg(&ese_dev->spi->dev, "failed to write to SPI\n");
			goto err;
		}
		ubuf += block;
		bytes -= block;
	}
	ret = len;
err:
	mutex_unlock(&ese_dev->mutex);
	return ret;
}

static ssize_t ese_read(struct file *filp, char __user *ubuf,
				size_t len, loff_t *offset)
{
	struct ese_dev *ese_dev = filp->private_data;
	ssize_t ret = -EFAULT;
	size_t bytes = len;
	char rx_buf[ST54_MAX_BUF];

	if (len > INT_MAX)
		return -EINVAL;
	mutex_lock(&ese_dev->mutex);
	while (bytes > 0) {
		size_t block = bytes < sizeof(rx_buf) ? bytes : sizeof(rx_buf);

		memset(rx_buf, 0, sizeof(rx_buf));
		ret = spi_read(ese_dev->spi, rx_buf, block);
		if (ret < 0) {
			dev_dbg(&ese_dev->spi->dev, "failed to read from SPI\n");
			goto err;
		}
		if (copy_to_user(ubuf, rx_buf, block)) {
			dev_dbg(&ese_dev->spi->dev, "failed to copy from user\n");
			goto err;
		}
		ubuf += block;
		bytes -= block;
	}
	ret = len;
err:
	mutex_unlock(&ese_dev->mutex);
	return ret;
}

static const struct file_operations ese_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read  = ese_read,
	.write = ese_write,
	.open = ese_open,
	.release = ese_release,
	.unlocked_ioctl = ese_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = ese_compat_ioctl
#endif
};

static int st54j_probe(struct spi_device *spi)
{
	struct ese_dev *ese_dev;
	struct device_node *np = dev_of_node(&spi->dev);
	int ret;

	dev_dbg(&spi->dev, "%s entry\n", __func__);

	if (!np) {
		dev_err(&spi->dev, "%s: device tree data missing\n", __func__);
		return -EINVAL;
	}

	ese_dev = kzalloc(sizeof(*ese_dev), GFP_KERNEL);
	if (ese_dev == NULL)
		return -ENOMEM;

	ese_dev->spi = spi;
	ese_dev->device.minor = MISC_DYNAMIC_MINOR;
	ese_dev->device.name = "st54j_se";
	ese_dev->device.fops = &ese_dev_fops;

	spi->bits_per_word = 8;

	ese_dev->ese_reset_gpio = of_get_named_gpio(np, "st,ese_reset_gpio", 0);
	if (!gpio_is_valid(ese_dev->ese_reset_gpio)) {
		pr_err("[dsc]%s: fail to get ese_reset_gpio\n", __func__);
		return -EINVAL;
		goto err;
	}
	pr_err("[dsc]%s : get ese_reset_gpio[%d]\n", __func__, ese_dev->ese_reset_gpio);

	ret = gpio_request(ese_dev->ese_reset_gpio, "ese_reset_gpio");
	if (ret) {
		pr_err("%s : ese_reset_gpio gpio_request failed\n", __FILE__);
		ret = -ENODEV;
		goto err;
	}
	
	/* Configure as output, initial low state */
	ret = gpio_direction_output(ese_dev->ese_reset_gpio, 0);
	if (ret) {
		pr_err("%s : ese_reset_gpio gpio_direction_output failed\n",
		       __FILE__);
		ret = -ENODEV;
		goto err;
	}

	mutex_init(&ese_dev->mutex);
	ret = of_property_read_string(np, "st,nfcc", &ese_dev->nfcc_name);
	if (IS_ERR_VALUE(ret)) {
		dev_err(&spi->dev,
			"%s: st,nfcc invalid or missing in device tree (%d)\n",
			__func__, ret);
		goto err;
	}
	dev_info(&spi->dev, "%s: device tree set '%s' as eSE power controller\n",
		__func__, ese_dev->nfcc_name);

	ret = misc_register(&ese_dev->device);
	if (ret) {
		dev_err(&spi->dev, "%s: misc_register failed\n", __func__);
		goto err;
	}
	dev_info(&spi->dev, "%s: eSE is configured\n", __func__);
	spi_set_drvdata(spi, ese_dev);

	return 0;
err:
	mutex_destroy(&ese_dev->mutex);
	kfree(ese_dev);
	return ret;
}

static int st54j_remove(struct spi_device *spi)
{
	struct ese_dev *ese_dev = spi_get_drvdata(spi);
	int ret = 0;

	if (!ese_dev) {
		dev_err(&spi->dev,
		"%s: device doesn't exist anymore\n", __func__);
		ret = -ENODEV;
		goto err;
	}
	/* If we have a NFC device, release it. */
	if (ese_dev->nfcc_device) {
		put_device(ese_dev->nfcc_device);
		ese_dev->nfcc_data = NULL;
		ese_dev->nfcc_device = NULL;
	}
	misc_deregister(&ese_dev->device);
	mutex_destroy(&ese_dev->mutex);
	kfree(ese_dev);
err:
	return ret;
}

static const struct of_device_id st54j_match_table[] = {
	{ .compatible = "st,st54j_se" },
	{ }
};
MODULE_DEVICE_TABLE(of, st54j_match_table);

static struct spi_driver st54j_driver = {
	.driver = {
		.name = "st54j_se",
		.of_match_table = st54j_match_table,
	},
	.probe = st54j_probe,
	.remove = st54j_remove,
};
module_spi_driver(st54j_driver);

MODULE_DESCRIPTION("ST54J eSE driver");
MODULE_ALIAS("spi:st54j_se");
MODULE_AUTHOR("ST Microelectronics");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
