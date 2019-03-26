/*
 * Copyright (C) 2013 ST Microelectronics S.A.
 * Copyright (C) 2010 Stollmann E+V GmbH
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef _ST21NFC_H_
#define _ST21NFC_H_

/*
 * The platform data member 'polarity_mode' defines
 * how the wakeup pin is configured and handled.
 * it can take the following values :
 *	 IRQF_TRIGGER_RISING
 *   IRQF_TRIGGER_HIGH
 */
struct st21nfc_platform_data {
	unsigned int irq_gpio;		/* GPIO for NFCC IRQ pin (input) */
	unsigned int reset_gpio;	/* GPIO for NFCC Reset pin (output) */
	unsigned int clkreq_gpio;	/* GPIO for NFCC CLK_REQ pin (input) */
	unsigned int pidle_gpio; 	/* GPIO for NFCC CLF_MONITOR_PWR (input) */
	unsigned int polarity_mode; 	/* irq_gpio polarity to be used */
};

#endif
