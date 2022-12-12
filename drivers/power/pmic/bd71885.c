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

static int bd71885_bind(struct udevice *dev)
{
	return bdxxxx_bind(dev, pmic_children_info);
}

static int bd71885_reg_count(struct udevice *dev)
{
	return BD71885_MAX_REGISTER - 1;
}

static int bd718xx_init_press_duration(struct udevice *ud)
{
	struct device* dev = bd718xx->chip.dev;
	uint short_press_ms, long_press_ms;
	uint short_press_value, long_press_value;
	int ret;

		/* TODO: Compare BD71837 and BS71885 spec and fix addresses/values.
		 * Convert to uBoot code
		 */
	ret =  dev_read_u32u(ud, "rohm,short-press-ms", &short_press_ms)
	if (!ret) {
		short_press_value = min(15u, (short_press_ms + 250) / 500);
		ret = regmap_update_bits(bd718xx->chip.regmap,
					 BD718XX_REG_PWRONCONFIG0,
					 BD718XX_PWRBTN_PRESS_DURATION_MASK,
					 short_press_value);
		if (ret) {
			dev_err(dev, "Failed to init pwron short press\n");
			return ret;
		}
	}

	ret = of_property_read_u32(dev->of_node, "rohm,long-press-ms",
				      &long_press_ms);
	if (!ret) {
		long_press_value = min(15u, (long_press_ms + 500) / 1000);
		ret = regmap_update_bits(bd718xx->chip.regmap,
					 BD718XX_REG_PWRONCONFIG1,
					 BD718XX_PWRBTN_PRESS_DURATION_MASK,
					 long_press_value);
		if (ret) {
			dev_err(dev, "Failed to init pwron long press\n");
			return ret;
		}
	}

	return 0;
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

