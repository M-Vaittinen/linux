// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright 2022 ROHM Semiconductors
 *  Matti Vaittinen <mazziesaccount@gmail.com>
 */

#include <common.h>
#include <errno.h>
#include <dm.h>
#include <i2c.h>
#include <log.h>
#include <asm/global_data.h>
#include <power/pmic.h>
#include <power/regulator.h>
#include <power/bd71885.h>
#include "bdxxxx.h"

DECLARE_GLOBAL_DATA_PTR;

static const struct pmic_child_info pmic_children_info[] = {
	/* buck */
	{ .prefix = "b", .driver = BD71885_REGULATOR_DRIVER},
	/* ldo */
	{ .prefix = "l", .driver = BD71885_REGULATOR_DRIVER},
	{ },
};

#define BD71885_SHORT_PRESS_MASK	(BIT(0) | BIT(1))
#define BD71885_MID_PRESS_MASK		(BIT(2) | BIT(3))
#define BD71885_LONG_PRESS_MASK		(BIT(4) | BIT(5)) 

static int bd718xx_init_press_duration(struct udevice *ud)
{
	uint short_press_ms, long_press_ms, mid_press_ms;
	/* Press durations in mS */
	static uint short_durations[] = { 16, 30, 60, 120 };
	static uint mid_durations[] = { 500, 1000, 4000, 8000 };
	static uint long_durations[] = { 1000, 5000, 10000, 15000 };
	int ret, i;

	ret =  dev_read_u32u(ud, "rohm,short-press-ms", &short_press_ms);
	if (!ret) {
		for (i = 0; i < ARRAY_SIZE(short_durations); i++) {
			if (short_durations[i] == short_press_ms) {
				ret = pmic_clrsetbits(ud, BD71885_REG_PBTNCONFIG, BD71885_SHORT_PRESS_MASK, i);
				if (ret)
					return ret;
				break;
			}
		}
		if (i == ARRAY_SIZE(short_durations))
			pr_err("Bad short-press duration %d\n", short_press_ms);
	}

	ret = dev_read_u32u(ud, "rohm,mid-press-ms", &mid_press_ms);
	if (!ret) {
		for (i = 0; i < ARRAY_SIZE(mid_durations); i++) {
			if (mid_durations[i] == mid_press_ms) {
				ret = pmic_clrsetbits(ud, BD71885_REG_PBTNCONFIG, BD71885_MID_PRESS_MASK, i << 2);
				if (ret)
					return ret;
				break;
			}
		}
		if (i == ARRAY_SIZE(mid_durations))
			pr_err("Bad mid-press duration %d\n", mid_press_ms);
	}

	ret = dev_read_u32u(ud, "rohm,long-press-ms", &long_press_ms);
	if (!ret) {
		for (i = 0; i < ARRAY_SIZE(long_durations); i++) {
			if (long_durations[i] == long_press_ms) {
				ret = pmic_clrsetbits(ud, BD71885_REG_PBTNCONFIG, BD71885_LONG_PRESS_MASK, i << 4);
				if (ret)
					return ret;
				break;
			}
		}
		if (i == ARRAY_SIZE(long_durations))
			pr_err("Bad long-press duration %d\n", long_press_ms);
	}

	return 0;
}

static int bd71885_bind(struct udevice *dev)
{
	int ret;

	ret = bdxxxx_bind(dev, pmic_children_info);
	if (ret)
		return ret;

	return bd718xx_init_press_duration(dev);
}

static int bd71885_reg_count(struct udevice *dev)
{
	return BD71885_MAX_REGISTER - 1;
}

static int bd71885_probe(struct udevice *dev)
{
	debug("%s: '%s' probed\n", __func__, dev->name);

	return 0;
}

static struct dm_pmic_ops bd71885_ops = {
	.reg_count = bd71885_reg_count,
	.read = bdxxxx_read,
	.write = bdxxxx_write,
};

static const struct udevice_id bd71885_ids[] = {
	{ .compatible = "rohm,bd71885", },
	{ }
};

U_BOOT_DRIVER(pmic_bd71885) = {
	.name = "bd71885_pmic",
	.id = UCLASS_PMIC,
	.of_match = bd71885_ids,
	.bind = bd71885_bind,
	.probe = bd71885_probe,
	.ops = &bd71885_ops,
};


/* *********************** Commands ************************/

#include <command.h>

static struct udevice *currdev;

static int failure(int ret)
{
	printf("Error: %d (%s)\n", ret, errno_str(ret));

	return CMD_RET_FAILURE;
}



static void print_boot_reas(int reg)
{
	static const char * const reason[] = {
		"Power-on button press",
		"Cold-boot flag",
		"RTC_ALM0",
		"",
		"",
		"",
		"BTMP WARN",
		"Cold-reset flag",
	};
	int i;

	if (!(reg & BD71885_BOOTSRC_MASK))
		printf("No boot-up reason indicated by BOOTSRC_1\n");
	else
		printf("Reasons stored for previous boot-up:\n");

	for (i = 0; i < 8; i++)
		if ((1 << i) & reg & BD71885_BOOTSRC_MASK)
			printf("- %s\n", reason[i]);
}

static void print_reset_reas(int reg)
{
	static const char * const reason[] = {
		"power-button long-press",
		"watchdog",
		"SW-reset request",
		"Thermal Shutdown",
		"Vsys UVLO or OVP",
		"EXTEN_OUT communication failure",
		"VR-Fault",
		"FAULT_B assertion",
	};
	int i;

	if (!reg)
		printf("No reset reason indicated by RESETSRC_1\n");
	else
		printf("Reasons stored for previous reset:\n");

	for (i = 0; i < 8; i++)
		if ((1 << i) & reg)
			printf("- %s\n", reason[i]);
}

static int print_faulty_vr(void)
{
	int buck, ldo, i;

	buck = pmic_reg_read(currdev, BD71885_RESETSRC2);
	if (buck)
		return buck;

	for (i = 0; i < 8; i++)
		if (!(buck & (1 << i)))
			printf("\tBUCK%d OK\n", i + 1);
		else
			printf("\tBUCK%d FAULTY\n", i + 1);

	ldo = pmic_reg_read(currdev, BD71885_RESETSRC3);
	if (ldo)
		return ldo;

	for (i = 0; i < 4; i++)
		if (!(ldo & (1 << i)))
			printf("\tLDO%d OK\n", i + 1);
		else
			printf("\tLDO%d FAULTY\n", i + 1);

	return 0;
}

/*
 * This function just hard-codes the BD71885 PMIC dt node-name and seeks
 * the udev based on this. It really is not the way to go - but as this
 * code is only used as a referene for SCF code building a nice u-boot
 * specific way of obtaining the device does not warrant the effort.
 *
 * TODO: Revise  this if the u-boot driver is ever to be sent upstream.
 * We could probably do:
 *	for (ret = uclass_first_device(UCLASS_PMIC, &dev); dev;
 *	     ret = uclass_next_device(&dev)) {
 *
 * and see if the driver is bd71885 driver. That should work for one piece
 * of BD71885 at a time. We could also allow user to specify the device to
 * access (similar to the pmic/regulator commands).
 */
static int get_dev(void)
{
//	char *name = "bd71885_pmic";
	char *name = "pmic@4b";
	int ret;

	if (!currdev) {
		ret = pmic_get(name, &currdev);
		if (ret) {
			printf("Can't get PMIC: %s!\n", name);
			return ret;
		}

		printf("dev: %d @ %s\n", dev_seq(currdev), currdev->name);
	}

	return 0;
}

/* Get the reset reason */
static int do_rr(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int ret;

	ret = get_dev();
	if (ret)
		return failure(ret);

	ret = pmic_reg_read(currdev, BD71885_RESETSRC1);
	if (ret < 0)
		return failure(ret);

	print_reset_reas(ret);
	if (ret & BD71885_VR_FAULT)
		ret = print_faulty_vr();

	if (ret)
		return failure(ret);

	return CMD_RET_SUCCESS;
}

/* Get the power-on reason */
static int do_pr(struct cmd_tbl *cmdtp, int flag, int argc,
		   char *const argv[])
{
	int ret;

	ret = get_dev();
	if (ret)
		return failure(ret);

	ret = pmic_reg_read(currdev, BD71885_BOOTSRC);
	if (ret < 0)
		return failure(ret);

	print_boot_reas(ret);

	return CMD_RET_SUCCESS;
}

static struct cmd_tbl subcmd[] = {
	U_BOOT_CMD_MKENT(rr, 1, 1, do_rr, "", ""),
	U_BOOT_CMD_MKENT(pr, 1, 1, do_pr, "", ""),
};

static int do_bd71885(struct cmd_tbl *cmdtp, int flag, int argc,
		   char *const argv[])
{
	struct cmd_tbl *cmd;

	argc--;
	argv++;

	cmd = find_cmd_tbl(argv[0], subcmd, ARRAY_SIZE(subcmd));
	if (cmd == NULL || argc > cmd->maxargs)
		return CMD_RET_USAGE;

	return cmd->cmd(cmdtp, flag, argc, argv);
}

U_BOOT_CMD(bd71885, CONFIG_SYS_MAXARGS, 1, do_bd71885,
	"BD71885 sub-system",
	"bd71885 rr - show previous reset reason\n"
	"bd71885 pr - show previous power-on reason\n"
);

