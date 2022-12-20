// SPDX-License-Identifier: GPL-2.0
/*
 *  Copyright 2022 ROHM Semiconductors
 *  Matti Vaittinen <mazziesaccount@gmail.com>
 */

#include <common.h>
#include <div64.h>
#include <dm.h>
#include <errno.h>
#include <i2c.h>
#include <log.h>
#include <asm/global_data.h>
#include <linux/delay.h>
#include <power/pmic.h>
#include <power/regulator.h>
#include <power/bd71885.h>
#include "bdxxxx.h"

DECLARE_GLOBAL_DATA_PTR;

/* Board specific sense resistor value read from the device-tree */
static uint g_r_sense;

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
	pr_err("Error: %d (%s)\n", ret, errno_str(ret));

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
					pr_err("Writing short_press failed %d\n", ret);
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
					pr_err("Writing mid_press failed %d\n", ret);
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
					pr_err("Writing long_press failed %d\n", ret);
					return ret;
				}
				break;
			}
		}
		if (i == ARRAY_SIZE(long_durations))
			pr_err("Bad long-press duration %d\n", long_press_ms);
	}
	ret =  dev_read_u32u(ud, "rohm,sense-resistor-ohms", &g_r_sense);
	if (!ret)
		printf("R_sense set to %u Ohms\n", g_r_sense);
	else
		pr_err("No sense resistor value found\n");

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
			pr_err("Can't get PMIC: %s!\n", name);
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
		pr_err("Failed to configure power-button (%d)\n", ret);
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
			pr_err("Don't know how to transfer '%s' => IDLE\n",
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
			pr_err("Don't know how to transfer '%s' => RUN\n",
			       state2txt(ret));
			ret = -EINVAL;
		}
	} else {
		pr_err("Unkown state '%s'. Valid ones are run, idle, suspend\n",
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
#define BD71885_ADC_ACCUM_VAL_BASE	0x74
#define BD71885_ADC_ACCUM_CNT_BASE	0x71

#define BD71885_ADC_CTRL_1 		0x6b
#define BD71885_ADC_CTRL_2 		0x6c

#define BD71885_ADC_ACCUM_KICK		0x70
#define BD71885_ADC_ACCUM_VOL_SRC 	0x0f
#define BD71885_ADC_ACCUM_SRC 		(BIT(5) | BIT(4))
#define BD71885_ADC_ACCUM_SRC_SIFT	4

#define BD71885_ADC_INTERVAL_MASK	(BIT(2) | BIT(1) | BIT(0))
#define BD71885_ADC_GAIN_MASK		(BIT(3) | BIT(4))
#define BD71885_ADC_GAIN_SIFT		3

#define BD71885_ADC_ACCUM_CLEAR BIT(2)
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
		pr_err("Bad ADC accum voltage source %d\n", ret);

		return -EINVAL;
	}

	return ret;
}


static int get_adc_accum(void)
{
	int ret;

	ret = pmic_reg_read(currdev, BD71885_ADC_ACCUM_KICK);

	if (ret < 0) {
		pr_err("Could not get ADC ACCUM state\n");

		return 0;
	}

	return ret & BD71885_ADC_ACCUM_START;
}


static int set_adc_accum(bool enable, bool clear)
{
	uint val = 0;

	if (clear)
		val |= BD71885_ADC_ACCUM_CLEAR;

	if (enable)
		val |= BD71885_ADC_ACCUM_START;
	else
		val |= BD71885_ADC_ACCUM_STOP;

	return pmic_reg_write(currdev, BD71885_ADC_ACCUM_KICK, val);
}

static int __stop_adc_accum(bool clear)
{
	int ret;

	ret = set_adc_accum(0, clear);

	if (ret)
		return ret;

	/* Ensure ADC is stopped prior returning */
	while (get_adc_accum())
		;

	return ret;
}

static int stop_adc_accum(void)
{
	return __stop_adc_accum(0);
}

static int stop_clear_adc_accum(void)
{
	return __stop_adc_accum(1);
}

static int start_adc_accum(void)
{
	return set_adc_accum(1, 0);
}

static int start_clear_adc_accum(void) __attribute__((unused));
static int start_clear_adc_accum(void)
{
	return set_adc_accum(1, 1);
}

static int do_adc_state(struct cmd_tbl *cmdtp, int flag, int argc,
			char *const argv[])
{
	char *state;
	int ret;

	ret = get_dev();
	if (ret)
		return failure(ret);

	if (argc == 1) {
		if (get_adc_accum())
			printf("ADC ACCUM is started\n");
		else
			printf("ADC ACCUM is stopped\n");

		return CMD_RET_SUCCESS;
	}

	if (argc != 2)
		return CMD_RET_USAGE;

	state = argv[1];

	if (!strcmp(state, "start"))
		ret = start_adc_accum();
	else if (!strcmp(state, "stop"))
		ret = stop_adc_accum();
	else
		return CMD_RET_USAGE;

	if (ret)
		return failure(ret);

	return CMD_RET_SUCCESS;
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
		pr_err("Bad ADC accum source %d\n", ret);

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

static const long bd71885_adc_gain[] = { 15, 30, 60, 100 };

static int __get_adc_gain(void)
{
	int ret;

	ret = pmic_reg_read(currdev, BD71885_ADC_CTRL_2);
	if (ret < 0)
		return ret;

	return ((ret & BD71885_ADC_GAIN_MASK) >> BD71885_ADC_GAIN_SIFT);
}

static int get_adc_gain(void)
{
	int ret;

	ret = __get_adc_gain();
	if (ret < 0)
		return failure(ret);

	printf("ADC gain set to '%ld'\n", bd71885_adc_gain[ret]);

	return CMD_RET_SUCCESS;
}

static int accum_stopped_config_helper(struct udevice *ud, uint reg, uint mask,
				       uint val)
{
	int started, ret;

	started = get_adc_accum();
	if (started) {
		ret = stop_adc_accum();

		if (ret)
			return failure(ret);
	}

	ret = pmic_clrsetbits(ud, reg, mask, val);
	if (ret) {
		if (started)
			pr_err("ADC config failed, ADC stopped\n");

		return ret;
	}

	if (started) {
		ret = start_adc_accum();
		if (ret)
			pr_err("Could not restart ADC\n");
	}

	return ret;
}

static int do_adc_gain(struct cmd_tbl *cmdtp, int flag, int argc,
		       char *const argv[])
{
	long gain;
	int ret, i;
	char *eptr;

	ret = get_dev();
	if (ret)
		return failure(ret);

	if (argc == 1)
		return get_adc_gain();

	if (argc != 2)
		return CMD_RET_USAGE;

        gain = simple_strtol(argv[1], &eptr, 10);
        if (!*argv[1] || *eptr)
		goto ret_usage;

	for (i = 0; i < ARRAY_SIZE(bd71885_adc_gain); i++)
		if (bd71885_adc_gain[i] == gain)
			break;

	if (i == ARRAY_SIZE(bd71885_adc_gain))
		goto ret_usage;

	ret = accum_stopped_config_helper(currdev, BD71885_ADC_CTRL_2,
					  BD71885_ADC_GAIN_MASK,
					  i << BD71885_ADC_GAIN_SIFT);
	if (ret)
		return failure(ret);

	return CMD_RET_SUCCESS;

ret_usage:
	pr_err("Bad gain. Should be one of [15, 30, 60, 100]\n");

	return CMD_RET_USAGE;
}

static int do_adc_source(struct cmd_tbl *cmdtp, int flag, int argc,
			 char *const argv[])
{
	char *src;
	int ret, reg;

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
		pr_err("Unsupported ADC accum source\n");

		return CMD_RET_USAGE;
	}

	ret = accum_stopped_config_helper(currdev, BD71885_ADC_CTRL_1,
					  BD71885_ADC_ACCUM_SRC, reg);
	if (ret)
		return failure(ret);

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

	ret = get_dev();
	if (ret)
		return failure(ret);

	if (argc == 1)
		return get_adc_vol_source();

	if (argc != 2)
		return CMD_RET_USAGE;

	if (ADC_SRC_VOLTAGE !=  __get_adc_source()) {
		pr_err("ADC ACCUM not set to accumulate voltage\n");
		get_adc_source();

		return failure(-EINVAL);
	}

	src = argv[1];

	for (i = 0; i < ARRAY_SIZE(adc_vol_sources); i++) {
		if (!strcmp(src, adc_vol_sources[i]))
			break;

	}
	if (i == ARRAY_SIZE(adc_vol_sources)) {
		pr_err("Unsupported ADC accum voltage source\n");

		return CMD_RET_USAGE;
	}

	ret = accum_stopped_config_helper(currdev, BD71885_ADC_CTRL_1,
					  BD71885_ADC_ACCUM_VOL_SRC, i);
	if (ret)
		return failure(ret);

	return CMD_RET_SUCCESS;
}

enum {
	TYPE_VOLTAGE,
	TYPE_CURRENT,
	TYPE_POWER,
	TYPE_TEMPERATURE,
	#define TYPE_MAX TYPE_TEMPERATURE
};

static int get_operation_type(char *arg)
{
	if (!arg)
		return -EINVAL;

	switch(arg[0])
	{
	case 'v':
		return TYPE_VOLTAGE;
	case 'i':
		return TYPE_CURRENT;
	case 'p':
		return TYPE_POWER;
	case 't':
		return TYPE_TEMPERATURE;
	}

	pr_err("Invalid operation\n");
	return -EINVAL;
}

enum {
	LIMIT_TYPE_ACCUM,
	LIMIT_TYPE_POWER,
	LIMIT_TYPE_TEMPERATURE,
	#define LIMIT_TYPE_MAX LIMIT_TYPE_TEMPERATURE
};

static int get_limit_type(char *arg)
{
	if (!arg)
		return -EINVAL;

	switch(arg[0])
	{
	case 'p':
		return LIMIT_TYPE_POWER;
	case 'a':
		return LIMIT_TYPE_ACCUM;
	case 't':
		return LIMIT_TYPE_TEMPERATURE;
	}

	pr_err("Invalid limit type\n");
	return -EINVAL;
}



#define BD71885_ADC_NUM_SAMPLES_BASE	0x6d
#define BD71885_MAX_ADC_SAMPLES 0x3fffff

static int bd71885_adc_set_num_samples(struct udevice *ud, long samples)
{
	int val = cpu_to_be32(samples) << 8;

	return pmic_write(ud, BD71885_ADC_NUM_SAMPLES_BASE, (char *)&val, 3);
}

static int interval2reg(long interval)
{
	static const int ivals[] = { 50, 100, 1000, 10000, 100000, 1000000};
	int i;

	for (i = 0; i < ARRAY_SIZE(ivals); i++)
		if (ivals[i] == interval)
			return i;

	return -EINVAL;
}

static int get_adc_accum_avg(struct udevice *ud, uint64_t *avg_value, uint64_t *accum)
{
	uint64_t *tmp2, val2;
	char buf[4], buf2[8];
	uint num_samples;
	u32 *tmp;
	int ret;

	tmp = (u32 *)&buf[0];

	ret = pmic_read(ud, BD71885_ADC_ACCUM_CNT_BASE, &buf[1], 3);
	if (ret)
		return ret;

	num_samples = be32_to_cpu(*tmp);

	ret = pmic_read(ud, BD71885_ADC_ACCUM_VAL_BASE, &buf2[3], 5);
	if (ret)
		return ret;

	tmp2 = (uint64_t *)&buf2[0];
	val2 = be64_to_cpu(*tmp2);

	*accum = val2;
	val2 += num_samples / 2;
	do_div(val2, num_samples);
	*avg_value = val2;

	return 0;
}

static int get_avg_voltage(struct udevice *ud, uint64_t *value, uint64_t *accum)
{
	int ret;

	ret = get_adc_accum_avg(ud, value, accum);
	if (ret)
		return ret;

	/* Scale? */
	return 0;
}

static int get_avg_current(struct udevice *ud, uint64_t *value, uint64_t *accum)
{
	int ret;

	ret = get_adc_accum_avg(ud, value, accum);
	if (ret)
		return ret;

	/* Scale? */
	return 0;
}

static int get_avg_power(struct udevice *ud, uint64_t *value, uint64_t *accum)
{
	int ret;

	ret = get_adc_accum_avg(ud, value, accum);
	if (ret)
		return ret;

	/* Scale? */
	return 0;
}

static int measure_avg(struct udevice *ud, int type, long samples,
		       long interval)
{
	unsigned long tmp = interval, meas_time;
	int multiplier = 0;
	int ret, i, j;
	int ireg;

	ireg = interval2reg(interval);
	ret = stop_clear_adc_accum();
	if (ret)
		return failure(ret);

	ret = pmic_clrsetbits(ud, BD71885_ADC_CTRL_2, BD71885_ADC_INTERVAL_MASK,
			      ireg);
	if (ret)
		return failure(ret);

	ret = bd71885_adc_set_num_samples(ud, samples);
	if (ret)
		return failure(ret);

	while (tmp > 1000) {
		tmp /= 10;
		multiplier++;
	}

	meas_time = samples * tmp;

	ret = start_adc_accum();
	if (ret)
		return failure(ret);

	/*
	 * Here we should catch the IRQ but for the sake of the simplicity
	 * we just sleep/delay for the time it takes to complete measurement.
	 *
	 * Note, we keep the CPU busy. This should probably be avoided in
	 * the product code using IRQs instead.
	 */
	if (!multiplier)
		udelay(meas_time);
	else
		for (j = 1; j < multiplier; j++)
			for (i = 0; i < 10; i++)
				udelay(meas_time);

	switch (type) {
	uint64_t avg, accum;

	case TYPE_VOLTAGE:
		ret = get_avg_voltage(ud, &avg, &accum);
		printf("Samples %lu, interval %lu, average voltage %llu, accumulated %llu\n",
		       samples, interval, avg, accum);
		break;
	case TYPE_CURRENT:
		ret = get_avg_current(ud, &avg, &accum);
		printf("Samples %lu, interval %lu, average current %llu accumulated %llu\n",
		       samples, interval, avg, accum);
		break;
	case TYPE_POWER:
		ret = get_avg_power(ud, &avg, &accum);
		printf("Samples %lu, interval %lu, average power %llu accumulated %llu\n",
		       samples, interval, avg, accum);
		break;
	default:
		pr_err("Unknown type\n");
		ret = -EINVAL;
		break;
	}
	if (ret)
		return failure(ret);

	return CMD_RET_SUCCESS;
}

static int do_adc_meas(struct cmd_tbl *cmdtp, int flag, int argc,
			     char *const argv[])
{
	long samples, interval;
	int type, ret;
	char *eptr;

	if (argc != 4)
		return CMD_RET_USAGE;

	type = get_operation_type(argv[1]);
	if (type < 0 || type == TYPE_TEMPERATURE)
		return CMD_RET_USAGE;

        samples = simple_strtol(argv[2], &eptr, 10);
        if (!*argv[2] || *eptr || ((unsigned long)samples) >= BD71885_MAX_ADC_SAMPLES)
		return CMD_RET_USAGE;

        interval = simple_strtol(argv[3], &eptr, 10);
        if (!*argv[3] || *eptr)
		return CMD_RET_USAGE;

	if (0 > interval2reg(interval))
		return CMD_RET_USAGE;

	ret = get_dev();
	if (ret)
		return failure(ret);

	return measure_avg(currdev, type, samples, interval);
}
#define BD71885_ADC_ACCUM_TH_BASE	0x81
#define BD71885_ADC_TEMP_WARN_BASE	0x86
#define BD71885_ADC_POWER_WARN_BASE	0x88

static int adc_set_accum_limit(struct udevice *ud, uint64_t limit)
{
	char *tmp;

	if (limit > 0x3FFFFFFFFULL) {
		pr_err("Limit too big\n");
		return -EINVAL;
	}

	limit = cpu_to_be64(limit);
	tmp = ((char*)&limit);

	return pmic_write(ud, BD71885_ADC_ACCUM_TH_BASE, &tmp[3], 5);
}

static int adc_set_power_limit(struct udevice *ud, uint64_t limit)
{
	char *tmp;
	u16 lim;

	if (limit > 0x3FFF) {
		pr_err("Limit too big\n");
		return -EINVAL;
	}

	lim = cpu_to_be16((u16)limit);
	tmp = ((char*)&lim);

	return pmic_write(ud, BD71885_ADC_POWER_WARN_BASE, &tmp[0], 2);

}

static int adc_set_temp_limit(struct udevice *ud, uint64_t limit)
{
	char *tmp;
	u16 lim;

	if (limit > 0x3ff) {
		pr_err("Limit too big\n");
		return -EINVAL;
	}

	lim = cpu_to_be16((u16)limit);
	tmp = ((char*)&lim);

	return pmic_write(ud, BD71885_ADC_TEMP_WARN_BASE, &tmp[0], 2);

}

static int do_adc_limit(struct cmd_tbl *cmdtp, int flag, int argc,
			     char *const argv[])
{
	long long limit;
	int type, ret;
	char *eptr;

	if (argc != 3)
		return CMD_RET_USAGE;

	type = get_limit_type(argv[1]);
	if (type < 0)
		return CMD_RET_USAGE;

        limit = simple_strtoll(argv[2], &eptr, 10);
        if (!*argv[2] || *eptr)
		return CMD_RET_USAGE;

	ret = get_dev();
	if (ret)
		return failure(ret);

	switch (type) {
	case LIMIT_TYPE_ACCUM:
		ret = adc_set_accum_limit(currdev, limit);
		break;
	case LIMIT_TYPE_POWER:
		ret = adc_set_power_limit(currdev, limit);
		break;
	case LIMIT_TYPE_TEMPERATURE:
		ret = adc_set_temp_limit(currdev, limit);
		break;
	default:
		pr_err("Unknown type\n");
		ret = -EINVAL;
		break;
	}
	if (ret)
		return failure(ret);

	return CMD_RET_SUCCESS;
}

static int do_adc_get(struct cmd_tbl *cmdtp, int flag, int argc,
			     char *const argv[])
{
	int type, ret;

	if (argc != 2)
		return CMD_RET_USAGE;

	type = get_operation_type(argv[1]);
	if (type < 0)
		return CMD_RET_USAGE;

	ret = get_dev();
	if (ret)
		return failure(ret);

	return CMD_RET_SUCCESS;
}

static struct cmd_tbl subcmd[] = {
	U_BOOT_CMD_MKENT(dt_init, 1, 1, do_dt_init, "", ""),
	U_BOOT_CMD_MKENT(rr, 1, 1, do_rr, "", ""),
	U_BOOT_CMD_MKENT(pr, 1, 1, do_pr, "", ""),
	U_BOOT_CMD_MKENT(hibernate, 1, 1, do_hibernate, "", ""),
	U_BOOT_CMD_MKENT(get_state, 1, 1, do_get_state, "", ""),
	U_BOOT_CMD_MKENT(set_state, 2, 1, do_set_state, "", ""),
	U_BOOT_CMD_MKENT(adc_state, 2, 1, do_adc_state, "", ""),
	U_BOOT_CMD_MKENT(adc_source, 2, 1, do_adc_source, "", ""),
	U_BOOT_CMD_MKENT(adc_vol_source, 2, 1, do_adc_vol_source, "", ""),
	U_BOOT_CMD_MKENT(adc_gain, 2, 1, do_adc_gain, "", ""),
	U_BOOT_CMD_MKENT(adc_meas, 4, 1, do_adc_meas, "", ""),
	U_BOOT_CMD_MKENT(adc_limit, 3, 1, do_adc_limit, "", ""),
	U_BOOT_CMD_MKENT(adc_get, 2, 1, do_adc_get, "", ""),
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
	"bd71885 adc_state - get or set ADC accum state (start, stop)\n"
	"bd71885 adc_source - get or set ADC accum source (voltage, current, power)\n"
	"bd71885 adc_vol_source - get or set ADC accum voltage source\n"
	"bd71885 adc_gain - get or set gain for ADC current accumulator\n"
	"bd71885 adc_meas [v, i, p] <num_samples> <interval> - measure\n"
	"bd71885 adc_limit [a(ccum), p(ower), t(emperature)] <threshold value> - set limit\n"
	"bd71885 adc_get [v, i, p, t] - get last measured value\n"
);

