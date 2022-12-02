/* SPDX-License-Identifier: GPL-2.0 */
/* Copyright (C) 2022 ROHM Semiconductors */

#ifndef BD718XX_H_
#define BD718XX_H_

#define BD2659_REGULATOR_DRIVER		"bd2659_regulator"
#define BD2659_VENDOR_ROHM		0x1f
#define BD2659_REV_KNOWN		0xa1

enum {
	BD2659_BUCK0_ID			= 0,
	BD2659_BUCK1_ID,
	BD2659_BUCK2_ID,
	BD2659_BUCK3_ID,
	BD2659_REGULATOR_AMOUNT
};

/*
 * The BUCK control registers are 4 bytes away from each others. Eg,
 * when we access some BUCK0 control for XXX from address Y, then buck1
 * control for XXX will be located at address Y + 4. Buck2 Y + 8, etc...
 */
#define BD2659_BUCK_CTRL_OFFSET		4
#define TO_BUCKx_REG(id, base_reg) ((base_reg) + (id) * BD2659_BUCK_CTRL_OFFSET)

enum {
	BD2659_ID			= 0x00,
	BD2659_REV			= 0x01,
	BD2659_BUCK0_VID_S0		= 0x0a
	BD2659_BUCK0_VID_S3		= 0x0b
	BD2659_REGLOCK			= 0xA0,
	BD2659_REG_OTPVER		= 0xB0,
	BD2659_MAX_REGISTER,
};

#define BD2659_MASK_REGLOCK		0x1

#endif
