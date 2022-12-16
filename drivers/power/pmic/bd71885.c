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

#if 0
static int tmp_update_bits(struct udevice *ud, uint reg, uint mask, uint val)
{
	int ret;
	uint8_t valb;

	printf("in tmp_update_bits: ud = %p\n", ud);
	ret = bdxxxx_read(ud, reg, &valb, 1);
	if (ret)
		return ret;

	valb &= ~((u8)mask);
	valb |= (u8)(val & mask);

	return bdxxxx_write(ud, reg, &valb, 1);
}
#endif

static int bd71885_bind(struct udevice *dev)
{

	return bdxxxx_bind(dev, pmic_children_info);
}

static int bd71885_reg_count(struct udevice *dev)
{
	return BD71885_MAX_REGISTER - 1;
}

static int bd71885_probe(struct udevice *dev)
{
	debug("%s: '%s' probed\n", __func__, dev->name);
	printf("%s: '%s' probed\n", __func__, dev->name);

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
		printf("Found rohm,short-press-ms %d\n", short_press_ms);
		for (i = 0; i < ARRAY_SIZE(short_durations); i++) {
			if (short_durations[i] == short_press_ms) {
				printf("writing %u to 0x%x\n", i, BD71885_REG_PBTNCONFIG);
				ret = pmic_clrsetbits(ud, BD71885_REG_PBTNCONFIG, BD71885_SHORT_PRESS_MASK, i);
				if (ret) {
					printf("Writing short_press failed %d\n", ret);
					return ret;
				}
				break;
			}
		}
		if (i == ARRAY_SIZE(short_durations))
			pr_err("Bad short-press duration %d\n", short_press_ms);
	}

	ret = dev_read_u32u(ud, "rohm,mid-press-ms", &mid_press_ms);
	if (!ret) {
		printf("Found rohm,mid-press-ms %d\n", mid_press_ms);
		for (i = 0; i < ARRAY_SIZE(mid_durations); i++) {
			if (mid_durations[i] == mid_press_ms) {
				printf("writing %u to 0x%x\n", i << 2, BD71885_REG_PBTNCONFIG);
				ret = pmic_clrsetbits(ud, BD71885_REG_PBTNCONFIG, BD71885_MID_PRESS_MASK, i << 2);
				if (ret) {
					printf("Writing mid_press failed %d\n", ret);
					return ret;
				}
				break;
			}
		}
		if (i == ARRAY_SIZE(mid_durations))
			pr_err("Bad mid-press duration %d\n", mid_press_ms);
	}

	ret = dev_read_u32u(ud, "rohm,long-press-ms", &long_press_ms);
	if (!ret) {
		printf("Found rohm,long-press-ms %d\n", long_press_ms);
		for (i = 0; i < ARRAY_SIZE(long_durations); i++) {
			if (long_durations[i] == long_press_ms) {
				printf("writing %u to 0x%x\n", i << 4, BD71885_REG_PBTNCONFIG);
				ret = pmic_clrsetbits(ud, BD71885_REG_PBTNCONFIG, BD71885_LONG_PRESS_MASK, i << 4);
				if (ret) {
					printf("Writing long_press failed %d\n", ret);
					return ret;
				}
				break;
			}
		}
		if (i == ARRAY_SIZE(long_durations))
			pr_err("Bad long-press duration %d\n", long_press_ms);
	}

	return 0;
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

static int do_dt_init(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int ret;

	ret = get_dev();
	if (ret)
		return failure(ret);

	ret = bd718xx_init_press_duration(currdev);
	if (ret) {
		printf("Failed to configure power-button (%d)\n", ret);
		return failure(ret);
	}

	return CMD_RET_SUCCESS;
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

static int do_hibernate(struct cmd_tbl *cmdtp, int flag, int argc, char *const argv[])
{
	int ret;

	ret = get_dev();
	if (ret)
		return failure(ret);

	/*
	 * This will effectively be cutting the power from the SOC. Calling code
	 * must have done the SOC specific preparations - which may include
	 * finishing writings, gating clocks, resets and so on to prevent data
	 * loss or HW damage
	 *
	 * INTLDO, RTC, Die temperature monitor, and Analog UVLO are powered
	 * during the HBNT State.
	 */
	printf("WARNING: PMIC going to hibernate - waiting for a turn-on event\n");

	ret = pmic_clrsetbits(currdev, BD71885_REG_PS_CTRL_1, BD71885_HIBERNATE_MASK,
			      BD71885_HIBERNATE_MASK);
	if (ret < 0)
		return failure(ret);

	for (;;)
		;

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

#define is_idle(reg) (((reg) & BD71885_STATE_MASK) == BD71885_STATE_IDLE)
#define is_shdn(reg) (((reg) & BD71885_STATE_MASK) == BD71885_STATE_SHDN)
#define is_emerg(reg) (((reg) & BD71885_STATE_MASK) == BD71885_STATE_EMERG)
#define is_dead(reg) (((reg) & BD71885_STATE_MASK) == BD71885_STATE_DEAD)
#define is_otp(reg) (((reg) & BD71885_STATE_MASK) == BD71885_STATE_OTP)
#define is_hbnt(reg) (((reg) & BD71885_STATE_MASK) == BD71885_STATE_HBNT)
#define is_run(reg) (((reg) & BD71885_STATE_MASK) == BD71885_STATE_RUN)
#define is_idle(reg) (((reg) & BD71885_STATE_MASK) == BD71885_STATE_IDLE)
#define is_suspend(reg) (((reg) & BD71885_STATE_MASK) == BD71885_STATE_SUSPEND)

static const char *state2txt(int reg)
{
	switch(reg & BD71885_STATE_MASK) {
	case BD71885_STATE_SHDN:
		return "SHUTDOWN";
	case BD71885_STATE_EMERG:
		return "EMERGENCY";
	case BD71885_STATE_DEAD:
		return "DEAD";
	case BD71885_STATE_OTP:
		return "OTP-LOAD";
	case BD71885_STATE_HBNT:
		return "HIBERNATE";
	case BD71885_STATE_RUN:
		return "RUN";
	case BD71885_STATE_IDLE:
		return "IDLE";
	case BD71885_STATE_SUSPEND:
		return "SUSPEND";
	default:
		break;
	}

	return "UNKNOWN";
}

static int gpio_set(int pin, int state)
{
	/* SOC specific code to toggle the GPIO pin */
	return 0;
}

static int do_set_state(struct cmd_tbl *cmdtp, int flag, int argc,
		   char *const argv[])
{
	char *state;
	int ret;

	ret = get_dev();
	if (ret)
		return failure(ret);

	if (argc != 2)
		return CMD_RET_USAGE;

	state = argv[1];

	ret = pmic_reg_read(currdev, BD71885_POWER_STATE);
	if (ret < 0)
		return failure(ret);

	if (!strcmp(state, "suspend")) {
		if (is_suspend(ret))
			return CMD_RET_SUCCESS;

		printf("PMIC going to SUSPEND\n");
		gpio_set(SUSPEND_GPIO_PIN, GPIO_HIGH);
	} else if (!strcmp(state, "idle")) {
		if (is_idle(ret))
			return CMD_RET_SUCCESS;

		if (is_run(ret) || is_suspend(ret)) {
			printf("PMIC %s => IDLE\n", state2txt(ret));
			/* Set IDLE mode */
			ret = pmic_clrsetbits(currdev, BD71885_REG_PS_CTRL_1,
					      BD71885_IDLE_MASK, BD71885_IDLE_MASK);
			if (is_suspend(ret))
				gpio_set(SUSPEND_GPIO_PIN, GPIO_LOW);
		} else {
			printf("Don't know how to transfer '%s' => IDLE\n",
			       state2txt(ret));
			ret = -EINVAL;
		}
	} else if (!strcmp(state, "run")) {
		if (is_run(ret))
			return CMD_RET_SUCCESS;

		if (is_idle(ret) || is_suspend(ret)) {
			printf("PMIC %s => RUN\n", state2txt(ret));
			/* Clear IDLE mode */
			ret = pmic_clrsetbits(currdev, BD71885_REG_PS_CTRL_1,
					      BD71885_IDLE_MASK, 0);
			if (is_suspend(ret))
				gpio_set(SUSPEND_GPIO_PIN, GPIO_LOW);
		} else {
			printf("Don't know how to transfer '%s' => RUN\n",
			       state2txt(ret));
			ret = -EINVAL;
		}
	} else {
		printf("Unkown state '%s'. Valid ones are run, idle, suspend\n",
		       state);
		ret = -EINVAL;
	}

	if (ret)
		return failure(ret);

	return CMD_RET_SUCCESS;
}

void print_power_state(int reg)
{
	const char *state;

	state = state2txt(reg);
	printf("Current power-state %s\n", state);
}

static int do_get_state(struct cmd_tbl *cmdtp, int flag, int argc,
		   char *const argv[])
{
	int ret;

	ret = get_dev();
	if (ret)
		return failure(ret);

	ret = pmic_reg_read(currdev, BD71885_POWER_STATE);
	if (ret < 0)
		return failure(ret);

	print_power_state(ret);

	return CMD_RET_SUCCESS;
}

#define BD71885_ADC_CTRL_1 		0x6b
#define BD71885_ADC_ACCUM_KICK		0x70
#define BD71885_ADC_ACCUM_SRC 		(BIT(5) | BIT(4))
#define BD71885_ADC_ACCUM_VOL_SRC 	0x0f
#define BD71885_ADC_ACCUM_SRC_SIFT	4

#define BD71885_ADC_ACCUM_STOP BIT(1)
#define BD71885_ADC_ACCUM_START BIT(0)

#define ADC_SRC_VOLTAGE			0
static int __get_adc_vol_source(void)
{
	int ret;

	ret = pmic_reg_read(currdev, BD71885_ADC_CTRL_1);
	if (ret < 0)
		return ret;

	ret &= BD71885_ADC_ACCUM_VOL_SRC;

	if (ret >= 0x0f) {
		printf("Bad ADC accum voltage source %d\n", ret);

		return -EINVAL;
	}

	return ret;
}


static int get_adc_accum(void)
{
	int ret;

	ret = pmic_reg_read(currdev, BD71885_ADC_ACCUM_KICK);

	if (ret < 0) {
		printf("Could not get ADC ACCUM state\n");

		return 0;
	}

	return ret & BD71885_ADC_ACCUM_START;
}


static int set_adc_accum(bool enable)
{
	if (enable)
		return pmic_reg_write(currdev, BD71885_ADC_ACCUM_KICK, BD71885_ADC_ACCUM_START);

	return pmic_reg_write(currdev, BD71885_ADC_ACCUM_KICK, BD71885_ADC_ACCUM_STOP);
}

static int stop_adc_accum(void)
{
	int ret;

	ret = set_adc_accum(0);

	if (ret)
		return ret;

	/* Ensure ADC is stopped prior returning */
	while (get_adc_accum())
		;

	return ret;
}

static int start_adc_accum(void)
{
	return set_adc_accum(1);
}

static int __get_adc_source(void)
{
	int ret;

	ret = pmic_reg_read(currdev, BD71885_ADC_CTRL_1);
	if (ret < 0)
		return ret;

	ret &= BD71885_ADC_ACCUM_SRC;
	ret >>= BD71885_ADC_ACCUM_SRC_SIFT;

	if (ret >= 3) {
		printf("Bad ADC accum source %d\n", ret);

		return -EINVAL;
	}

	return ret;
}

static int get_adc_source(void)
{
	static const char * const src[] = { "voltage", "current", "power" };
	int ret;

	ret = __get_adc_source();
	if (ret < 0)
		return failure(ret);

	printf("ADC accumulator set to count '%s'\n", src[ret]);

	return CMD_RET_SUCCESS;
}

static int do_adc_source(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	char *src;
	int ret, reg;
	int started;

	ret = get_dev();
	if (ret)
		return failure(ret);

	if (argc == 1)
		return get_adc_source();

	if (argc != 2)
		return CMD_RET_USAGE;

	src = argv[1];

	if (!strcmp(src, "voltage")) {
		reg = 0;
	} else if (!strcmp(src, "current")) {
		reg = 1 << BD71885_ADC_ACCUM_SRC_SIFT;
	} else if (!strcmp(src, "power")) {
		reg = 2 << BD71885_ADC_ACCUM_SRC_SIFT;
	} else {
		printf("Unsupported ADC accum source\n");

		return CMD_RET_USAGE;
	}

	started = get_adc_accum();
	if (started) {
		ret = stop_adc_accum();

		if (ret)
			return failure(ret);
	}

	ret = pmic_clrsetbits(currdev, BD71885_ADC_CTRL_1,
			      BD71885_ADC_ACCUM_SRC, reg);
	if (ret)
		return failure(ret);

	if (started) {
		ret = start_adc_accum();
		if (ret) {
			printf("Could not restart ADC\n");
			return failure(ret);
		}
	}

	return CMD_RET_SUCCESS;
}

static const char * const adc_vol_sources[] = {
	"vsys",
	"fb1",
	"fb2",
	"fb3",
	"fb4",
	"fb5",
	"fb6",
	"fb7",
	"fb8",
	"ldo1",
	"ldo2",
	"ldo3",
	"ldo4",
	"adin_p",
	"gpio1",
};

static int get_adc_vol_source(void)
{
	int ret = __get_adc_vol_source();

	if (ret < 0)
		return failure(ret);

	printf("ADC Accumulator voltage source set to '%s'\n",
	       adc_vol_sources[ret]);

	return CMD_RET_SUCCESS;
}

static int do_adc_vol_source(struct cmd_tbl *cmdtp, int flag, int argc,
			     char *const argv[])
{
	char *src;
	int ret, i;
	int started;

	ret = get_dev();
	if (ret)
		return failure(ret);

	if (argc == 1)
		return get_adc_vol_source();

	if (argc != 2)
		return CMD_RET_USAGE;

	if (ADC_SRC_VOLTAGE !=  __get_adc_source()) {
		printf("ADC ACCUM not set to accumulate voltage\n");
		get_adc_source();

		return failure(-EINVAL);
	}

	src = argv[1];

	for (i = 0; i < ARRAY_SIZE(adc_vol_sources); i++) {
		if (!strcmp(src, adc_vol_sources[i]))
			break;

	}
	if (i == ARRAY_SIZE(adc_vol_sources)) {
		printf("Unsupported ADC accum voltage source\n");

		return CMD_RET_USAGE;
	}

	started = get_adc_accum();
	if (started) {
		ret = stop_adc_accum();

		if (ret)
			return failure(ret);
	}

	ret = pmic_clrsetbits(currdev, BD71885_ADC_CTRL_1,
			      BD71885_ADC_ACCUM_VOL_SRC, i);
	if (ret)
		return failure(ret);

	if (started) {
		ret = start_adc_accum();
		if (ret) {
			printf("Could not restart ADC\n");
			return failure(ret);
		}
	}

	return CMD_RET_SUCCESS;
}

static struct cmd_tbl subcmd[] = {
	U_BOOT_CMD_MKENT(dt_init, 1, 1, do_dt_init, "", ""),
	U_BOOT_CMD_MKENT(rr, 1, 1, do_rr, "", ""),
	U_BOOT_CMD_MKENT(pr, 1, 1, do_pr, "", ""),
	U_BOOT_CMD_MKENT(hibernate, 1, 1, do_hibernate, "", ""),
	U_BOOT_CMD_MKENT(get_state, 1, 1, do_get_state, "", ""),
	U_BOOT_CMD_MKENT(set_state, 2, 1, do_set_state, "", ""),
	U_BOOT_CMD_MKENT(DC_SOURCE, 2, 1, do_adc_source, "", ""),
	U_BOOT_CMD_MKENT(DC_SOURCE, 2, 1, do_adc_vol_source, "", ""),
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
	"bd71885 hibernate - tranfer to HBNT state - MAY DAMAGE HW\n"
	"bd71885 get_state - read current power-state\n"
	"bd71885 set_state <state>- set power-state (suspend, idle, run)\n"
	"bd71885 dt_init - initialize PMIC based on DT values\n"
	"bd71885 adc_source - get or set ADC accum source (voltage, current, power)\n"
	"bd71885 adc_vol_source - get or set ADC accum voltage source\n"
);

