// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2019 ROHM Semiconductors
// bd71828-regulator.c ROHM BD71828GW-DS1 regulator driver
//

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/rohm-bd71828.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>

#define MAX_GPIO_DVS_BUCKS 4
#define DVS_RUN_LEVELS 4

struct reg_init {
	unsigned int reg;
	unsigned int mask;
	unsigned int val;
};

struct run_lvl_ctrl {
	unsigned int voltage;
	bool enabled;
};

struct bd71828_regulator_data {
	struct regulator_desc desc;
	struct rohm_dvs_config dvs;
	const struct reg_init *reg_inits;
	int reg_init_amnt;
	struct run_lvl_ctrl run_lvl[DVS_RUN_LEVELS];
	struct gpio_desc *gpio1;
	struct gpio_desc *gpio2;
};

static const struct reg_init buck1_inits[] = {
	/*
	 * DVS Buck voltages can be changed by register values or via GPIO.
	 * Use register accesses by default.
	 */
	{
		.reg = BD71828_REG_PS_CTRL_1,
		.mask = BD71828_MASK_DVS_BUCK1_CTRL,
		.val = BD71828_DVS_BUCK1_CTRL_I2C,
	},
};

static const struct reg_init buck1_gpio_inits[] = {
	{
		.reg = BD71828_REG_PS_CTRL_1,
		.mask = BD71828_MASK_DVS_BUCK1_CTRL,
		.val = BD71828_DVS_BUCK1_CTRL_GPIO,
	},
};

static const struct reg_init buck2_inits[] = {
	{
		.reg = BD71828_REG_PS_CTRL_1,
		.mask = BD71828_MASK_DVS_BUCK2_CTRL,
		.val = BD71828_DVS_BUCK2_CTRL_I2C,
	},
};

static const struct reg_init buck2_gpio_inits[] = {
	{
		.reg = BD71828_REG_PS_CTRL_1,
		.mask = BD71828_MASK_DVS_BUCK2_CTRL,
		.val = BD71828_DVS_BUCK2_CTRL_GPIO,
	},
};

static const struct reg_init buck6_inits[] = {
	{
		.reg = BD71828_REG_PS_CTRL_1,
		.mask = BD71828_MASK_DVS_BUCK6_CTRL,
		.val = BD71828_DVS_BUCK6_CTRL_I2C,
	},
};

static const struct reg_init buck6_gpio_inits[] = {
	{
		.reg = BD71828_REG_PS_CTRL_1,
		.mask = BD71828_MASK_DVS_BUCK6_CTRL,
		.val = BD71828_DVS_BUCK6_CTRL_GPIO,
	},
};

static const struct reg_init buck7_inits[] = {
	{
		.reg = BD71828_REG_PS_CTRL_1,
		.mask = BD71828_MASK_DVS_BUCK7_CTRL,
		.val = BD71828_DVS_BUCK7_CTRL_I2C,
	},
};

static const struct reg_init buck7_gpio_inits[] = {
	{
		.reg = BD71828_REG_PS_CTRL_1,
		.mask = BD71828_MASK_DVS_BUCK7_CTRL,
		.val = BD71828_DVS_BUCK7_CTRL_GPIO,
	},
};

static const struct regulator_linear_range bd71828_buck1267_volts[] = {
	REGULATOR_LINEAR_RANGE(500000, 0x00, 0xef, 6250),
	REGULATOR_LINEAR_RANGE(2000000, 0xf0, 0xff, 0),
};

static const struct regulator_linear_range bd71828_buck3_volts[] = {
	REGULATOR_LINEAR_RANGE(1200000, 0x00, 0x0f, 50000),
	REGULATOR_LINEAR_RANGE(2000000, 0x10, 0x1f, 0),
};

static const struct regulator_linear_range bd71828_buck4_volts[] = {
	REGULATOR_LINEAR_RANGE(1000000, 0x00, 0x1f, 25000),
	REGULATOR_LINEAR_RANGE(1800000, 0x20, 0x3f, 0),
};

static const struct regulator_linear_range bd71828_buck5_volts[] = {
	REGULATOR_LINEAR_RANGE(2500000, 0x00, 0x0f, 50000),
	REGULATOR_LINEAR_RANGE(3300000, 0x10, 0x1f, 0),
};

static const struct regulator_linear_range bd71828_ldo_volts[] = {
	REGULATOR_LINEAR_RANGE(800000, 0x00, 0x31, 50000),
	REGULATOR_LINEAR_RANGE(3300000, 0x32, 0x3f, 0),
};

static int bd71828_set_ramp_delay(struct regulator_dev *rdev, int ramp_delay)
{
	dev_err(&rdev->dev, "TODO: %s\n", __func__);
	return -ENOSYS;
}

static int buck_set_hw_dvs_levels(struct device_node *np,
			    const struct regulator_desc *desc,
			    struct regulator_config *cfg)
{
	struct bd71828_regulator_data *data;

	data = container_of(desc, struct bd71828_regulator_data, desc);

	return rohm_regulator_set_dvs_levels(&data->dvs, np, desc, cfg->regmap);
}

static int set_runlevel_voltage(struct regmap *regmap,
				const struct regulator_desc *desc,
				unsigned int uv, unsigned int level)
{
	int i, ret = -EINVAL;
	/*
	 * RUN level registers are next to vsel_reg. RUN0 reg is next, then
	 * is the RUN 1 reg and so on...
	 */
	u8 reg = desc->vsel_reg + level + 1;
	u8 mask = BD71828_MASK_BUCK1267_VOLT;

	for (i = 0; i < desc->n_voltages; i++) {
		ret = regulator_desc_list_voltage_linear_range(desc, i);
		if (ret < 0)
			continue;
		if (ret == uv) {
			i <<= ffs(desc->vsel_mask) - 1;
			ret = regmap_update_bits(regmap, reg, mask, i);
			break;
		}
	}
	return ret;
}

static int buck_set_gpio_hw_dvs_levels(struct device_node *np,
				       const struct regulator_desc *desc,
				       struct regulator_config *cfg)
{
	struct bd71828_regulator_data *data;
	uint32_t uv;
	int i, ret;
	/* On BD71828 the RUN level control reg is next to enable reg */
	u8 en_reg = desc->enable_reg + 1;
	const char *props[DVS_RUN_LEVELS] = { "rohm,dvs-runlevel0-voltage",
					      "rohm,dvs-runlevel1-voltage",
					      "rohm,dvs-runlevel2-voltage",
					      "rohm,dvs-runlevel3-voltage" };
	u8 en_masks[DVS_RUN_LEVELS] = { BD71828_MASK_RUN0_EN,
					BD71828_MASK_RUN1_EN,
					BD71828_MASK_RUN2_EN,
					BD71828_MASK_RUN3_EN };

	data = container_of(desc, struct bd71828_regulator_data, desc);

	for (i = 0; i < DVS_RUN_LEVELS; i++) {
		ret = of_property_read_u32(np, props[i], &uv);
		if (ret) {
			if (ret != -EINVAL)
				return ret;
			uv = 0;
		}
		if (uv) {
			data->run_lvl[i].voltage = uv;
			data->run_lvl[i].enabled = true;

			ret = set_runlevel_voltage(cfg->regmap, desc, uv, i);

			if (ret)
				return ret;

			ret = regmap_update_bits(cfg->regmap, en_reg,
						 en_masks[i], en_masks[i]);
		} else {
			ret = regmap_update_bits(cfg->regmap, en_reg,
						 en_masks[i], 0);
		}
		if (ret)
			return ret;
	}

	return rohm_regulator_set_dvs_levels(&data->dvs, np, desc, cfg->regmap);
}

static int ldo6_parse_dt(struct device_node *np,
			 const struct regulator_desc *desc,
			 struct regulator_config *cfg)
{
	int ret, i;
	uint32_t uv = 0;
	unsigned int en;
	struct regmap *regmap = cfg->regmap;
	const char *props[] = { "rohm,dvs-run-voltage", "rohm,dvs-idle-voltage",
				"rohm,dvs-suspend-voltage",
				"rohm,dvs-lpsr-voltage" };
	unsigned int mask[] = { BD71828_MASK_RUN_EN, BD71828_MASK_IDLE_EN,
		 	       BD71828_MASK_SUSP_EN, BD71828_MASK_LPSR_EN };

	for (i = 0; i < ARRAY_SIZE(props); i++) {
		ret = of_property_read_u32(np, props[i], &uv);
		if (ret) {
			if (ret != -EINVAL)
				return ret;
			continue;
		}
		if (uv)
			en = 0xffffffff;
		else
			en = 0;

		ret = regmap_update_bits(regmap, desc->enable_reg, mask[i], en);
		if (ret)
			return ret;
	}
	return 0;
}

static int bd71828_dvs_gpio_set_run_level(struct bd71828_regulator_data *rd,
					  int val)
{
	if (val < 0 || val > 3)
		return -EINVAL;

	gpiod_set_value_cansleep(rd->gpio1, val & 1);
	gpiod_set_value_cansleep(rd->gpio2, val & 2);

	return 0;
}
static int bd71828_dvs_gpio_get_run_level(struct bd71828_regulator_data *data)
{
	int run_level;
	int tmp;

	tmp = gpiod_get_value_cansleep(data->gpio1);
	if (tmp < 0)
		return tmp;

	run_level = tmp;

	tmp = gpiod_get_value_cansleep(data->gpio2);
	if (tmp < 0)
		return tmp;

	run_level |= (tmp << 1);

	return run_level;
}

static int bd71828_dvs_gpio_is_enabled(struct regulator_dev *rdev)
{
	struct bd71828_regulator_data *data = rdev_get_drvdata(rdev);
	int ret;

	// TODO: lock GPIO state (Is this needed)
	ret = bd71828_dvs_gpio_get_run_level(data);
	if (ret < 0)
		goto unlock_out;

	ret = data->run_lvl[ret].enabled;

unlock_out:
	//TODO: unlock

	return ret;
}

static int bd71828_dvs_gpio_get_voltage(struct regulator_dev *rdev)
{
	int ret;
	struct bd71828_regulator_data *data = rdev_get_drvdata(rdev);

	// TODO: lock GPIO state (Is this needed)
	ret = bd71828_dvs_gpio_get_run_level(data);
	if (ret < 0)
		goto unlock_out;

	ret = data->run_lvl[ret].voltage;

unlock_out:
	//TODO: unlock

	return ret;
}

static const struct regulator_ops dvs_buck_gpio_ops = {
	.is_enabled = bd71828_dvs_gpio_is_enabled,
	.get_voltage = bd71828_dvs_gpio_get_voltage,
};

static const struct regulator_ops bd71828_buck_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

static const struct regulator_ops bd71828_dvs_buck_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.set_ramp_delay = bd71828_set_ramp_delay,
};

static const struct regulator_ops bd71828_ldo_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
};

static int bd71828_ldo6_get_voltage (struct regulator_dev *rdev)
{
	return BD71828_LDO_6_VOLTAGE;
}

static const struct regulator_ops bd71828_ldo6_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.get_voltage = bd71828_ldo6_get_voltage,
	.is_enabled = regulator_is_enabled_regmap,
};

static const struct bd71828_regulator_data bd71828_rdata[] = {
	{
		.desc = {
			.name = "buck1",
			.of_match = of_match_ptr("BUCK1"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD71828_BUCK1,
			.ops = &bd71828_dvs_buck_ops,
			.type = REGULATOR_VOLTAGE,
			.linear_ranges = bd71828_buck1267_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71828_buck1267_volts),
			.n_voltages = BD71828_BUCK1267_VOLTS,
			.enable_reg = BD71828_REG_BUCK1_EN,
			.enable_mask = BD71828_MASK_RUN_EN,
			.vsel_reg = BD71828_REG_BUCK1_VOLT,
			.vsel_mask = BD71828_MASK_BUCK1267_VOLT,
			.owner = THIS_MODULE,
			.of_parse_cb = buck_set_hw_dvs_levels,
		},
		.dvs = {
			.level_map = ROHM_DVS_LEVEL_RUN | ROHM_DVS_LEVEL_IDLE |
				     ROHM_DVS_LEVEL_SUSPEND |
				     ROHM_DVS_LEVEL_LPSR,
			.run_reg = BD71828_REG_BUCK1_VOLT,
			.run_mask = BD71828_MASK_BUCK1267_VOLT,
			.idle_reg = BD71828_REG_BUCK1_IDLE_VOLT,
			.idle_mask = BD71828_MASK_BUCK1267_VOLT,
			.idle_on_mask = BD71828_MASK_IDLE_EN,
			.suspend_reg = BD71828_REG_BUCK1_SUSP_VOLT,
			.suspend_mask = BD71828_MASK_BUCK1267_VOLT,
			.suspend_on_mask = BD71828_MASK_SUSP_EN,
			.lpsr_on_mask = BD71828_MASK_LPSR_EN,
			/*
			 * LPSR voltage is same as SUSPEND voltage. Allow
			 * setting it so that regulator can be set enabled at
			 * LPSR state
			 */
			.lpsr_reg = BD71828_REG_BUCK1_SUSP_VOLT,
			.lpsr_mask = BD71828_MASK_BUCK1267_VOLT,
		},
		.reg_inits = buck1_inits,
		.reg_init_amnt = ARRAY_SIZE(buck1_inits),
	},
	{
		.desc = {
			.name = "buck2",
			.of_match = of_match_ptr("BUCK2"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD71828_BUCK2,
			.ops = &bd71828_dvs_buck_ops,
			.type = REGULATOR_VOLTAGE,
			.linear_ranges = bd71828_buck1267_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71828_buck1267_volts),
			.n_voltages = BD71828_BUCK1267_VOLTS,
			.enable_reg = BD71828_REG_BUCK2_EN,
			.enable_mask = BD71828_MASK_RUN_EN,
			.vsel_reg = BD71828_REG_BUCK2_VOLT,
			.vsel_mask = BD71828_MASK_BUCK1267_VOLT,
			.owner = THIS_MODULE,
			.of_parse_cb = buck_set_hw_dvs_levels,
		},
		.dvs = {
			.level_map = ROHM_DVS_LEVEL_RUN | ROHM_DVS_LEVEL_IDLE |
				     ROHM_DVS_LEVEL_SUSPEND |
				     ROHM_DVS_LEVEL_LPSR,
			.run_reg = BD71828_REG_BUCK2_VOLT,
			.run_mask = BD71828_MASK_BUCK1267_VOLT,
			.idle_reg = BD71828_REG_BUCK2_IDLE_VOLT,
			.idle_mask = BD71828_MASK_BUCK1267_VOLT,
			.idle_on_mask = BD71828_MASK_IDLE_EN,
			.suspend_reg = BD71828_REG_BUCK2_SUSP_VOLT,
			.suspend_mask = BD71828_MASK_BUCK1267_VOLT,
			.suspend_on_mask = BD71828_MASK_SUSP_EN,
			.lpsr_on_mask = BD71828_MASK_LPSR_EN,
			.lpsr_reg = BD71828_REG_BUCK2_SUSP_VOLT,
			.lpsr_mask = BD71828_MASK_BUCK1267_VOLT,
		},
		.reg_inits = buck2_inits,
		.reg_init_amnt = ARRAY_SIZE(buck2_inits),
	},
	{
		.desc = {
			.name = "buck3",
			.of_match = of_match_ptr("BUCK3"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD71828_BUCK3,
			.ops = &bd71828_buck_ops,
			.type = REGULATOR_VOLTAGE,
			.linear_ranges = bd71828_buck3_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71828_buck3_volts),
			.n_voltages = BD71828_BUCK3_VOLTS,
			.enable_reg = BD71828_REG_BUCK3_EN,
			.enable_mask = BD71828_MASK_RUN_EN,
			.vsel_reg = BD71828_REG_BUCK3_VOLT,
			.vsel_mask = BD71828_MASK_BUCK3_VOLT,
			.owner = THIS_MODULE,
			.of_parse_cb = buck_set_hw_dvs_levels,
		},
		.dvs = {
			/*
			 * BUCK3 only supports single voltage for all states.
			 * voltage can be individually enabled for each state
			 * though => allow setting all states to support
			 * enabling power rail on different states.
			 */
			.level_map = ROHM_DVS_LEVEL_RUN | ROHM_DVS_LEVEL_IDLE |
				     ROHM_DVS_LEVEL_SUSPEND |
				     ROHM_DVS_LEVEL_LPSR,
			.run_reg = BD71828_REG_BUCK3_VOLT,
			.idle_reg = BD71828_REG_BUCK3_VOLT,
			.suspend_reg = BD71828_REG_BUCK3_VOLT,
			.lpsr_reg = BD71828_REG_BUCK3_VOLT,
			.run_mask = BD71828_MASK_BUCK3_VOLT,
			.idle_mask = BD71828_MASK_BUCK3_VOLT,
			.suspend_mask = BD71828_MASK_BUCK3_VOLT,
			.lpsr_mask = BD71828_MASK_BUCK3_VOLT,
			.idle_on_mask = BD71828_MASK_IDLE_EN,
			.suspend_on_mask = BD71828_MASK_SUSP_EN,
			.lpsr_on_mask = BD71828_MASK_LPSR_EN,
		},
	},
	{
		.desc = {
			.name = "buck4",
			.of_match = of_match_ptr("BUCK4"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD71828_BUCK4,
			.ops = &bd71828_buck_ops,
			.type = REGULATOR_VOLTAGE,
			.linear_ranges = bd71828_buck4_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71828_buck4_volts),
			.n_voltages = BD71828_BUCK4_VOLTS,
			.enable_reg = BD71828_REG_BUCK4_EN,
			.enable_mask = BD71828_MASK_RUN_EN,
			.vsel_reg = BD71828_REG_BUCK4_VOLT,
			.vsel_mask = BD71828_MASK_BUCK4_VOLT,
			.owner = THIS_MODULE,
			.of_parse_cb = buck_set_hw_dvs_levels,
		},
		.dvs = {
			/*
			 * BUCK4 only supports single voltage for all states.
			 * voltage can be individually enabled for each state
			 * though => allow setting all states to support
			 * enabling power rail on different states.
			 */
			.level_map = ROHM_DVS_LEVEL_RUN | ROHM_DVS_LEVEL_IDLE |
				     ROHM_DVS_LEVEL_SUSPEND |
				     ROHM_DVS_LEVEL_LPSR,
			.run_reg = BD71828_REG_BUCK4_VOLT,
			.idle_reg = BD71828_REG_BUCK4_VOLT,
			.suspend_reg = BD71828_REG_BUCK4_VOLT,
			.lpsr_reg = BD71828_REG_BUCK4_VOLT,
			.run_mask = BD71828_MASK_BUCK4_VOLT,
			.idle_mask = BD71828_MASK_BUCK4_VOLT,
			.suspend_mask = BD71828_MASK_BUCK4_VOLT,
			.lpsr_mask = BD71828_MASK_BUCK4_VOLT,
			.idle_on_mask = BD71828_MASK_IDLE_EN,
			.suspend_on_mask = BD71828_MASK_SUSP_EN,
			.lpsr_on_mask = BD71828_MASK_LPSR_EN,
		},
	},
	{
		.desc = {
			.name = "buck5",
			.of_match = of_match_ptr("BUCK5"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD71828_BUCK5,
			.ops = &bd71828_buck_ops,
			.type = REGULATOR_VOLTAGE,
			.linear_ranges = bd71828_buck5_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71828_buck5_volts),
			.n_voltages = BD71828_BUCK5_VOLTS,
			.enable_reg = BD71828_REG_BUCK5_EN,
			.enable_mask = BD71828_MASK_RUN_EN,
			.vsel_reg = BD71828_REG_BUCK5_VOLT,
			.vsel_mask = BD71828_MASK_BUCK5_VOLT,
			.owner = THIS_MODULE,
			.of_parse_cb = buck_set_hw_dvs_levels,
		},
		.dvs = {
			/*
			 * BUCK5 only supports single voltage for all states.
			 * voltage can be individually enabled for each state
			 * though => allow setting all states to support
			 * enabling power rail on different states.
			 */
			.level_map = ROHM_DVS_LEVEL_RUN | ROHM_DVS_LEVEL_IDLE |
				     ROHM_DVS_LEVEL_SUSPEND |
				     ROHM_DVS_LEVEL_LPSR,
			.run_reg = BD71828_REG_BUCK5_VOLT,
			.idle_reg = BD71828_REG_BUCK5_VOLT,
			.suspend_reg = BD71828_REG_BUCK5_VOLT,
			.lpsr_reg = BD71828_REG_BUCK5_VOLT,
			.run_mask = BD71828_MASK_BUCK5_VOLT,
			.idle_mask = BD71828_MASK_BUCK5_VOLT,
			.suspend_mask = BD71828_MASK_BUCK5_VOLT,
			.lpsr_mask = BD71828_MASK_BUCK5_VOLT,
			.idle_on_mask = BD71828_MASK_IDLE_EN,
			.suspend_on_mask = BD71828_MASK_SUSP_EN,
			.lpsr_on_mask = BD71828_MASK_LPSR_EN,
		},
	},
	{
		.desc = {
			.name = "buck6",
			.of_match = of_match_ptr("BUCK6"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD71828_BUCK6,
			.ops = &bd71828_dvs_buck_ops,
			.type = REGULATOR_VOLTAGE,
			.linear_ranges = bd71828_buck1267_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71828_buck1267_volts),
			.n_voltages = BD71828_BUCK1267_VOLTS,
			.enable_reg = BD71828_REG_BUCK6_EN,
			.enable_mask = BD71828_MASK_RUN_EN,
			.vsel_reg = BD71828_REG_BUCK6_VOLT,
			.vsel_mask = BD71828_MASK_BUCK1267_VOLT,
			.owner = THIS_MODULE,
			.of_parse_cb = buck_set_hw_dvs_levels,
		},
		.dvs = {
			.level_map = ROHM_DVS_LEVEL_RUN | ROHM_DVS_LEVEL_IDLE |
				     ROHM_DVS_LEVEL_SUSPEND |
				     ROHM_DVS_LEVEL_LPSR,
			.run_reg = BD71828_REG_BUCK6_VOLT,
			.run_mask = BD71828_MASK_BUCK1267_VOLT,
			.idle_reg = BD71828_REG_BUCK6_IDLE_VOLT,
			.idle_mask = BD71828_MASK_BUCK1267_VOLT,
			.idle_on_mask = BD71828_MASK_IDLE_EN,
			.suspend_reg = BD71828_REG_BUCK6_SUSP_VOLT,
			.suspend_mask = BD71828_MASK_BUCK1267_VOLT,
			.suspend_on_mask = BD71828_MASK_SUSP_EN,
			.lpsr_on_mask = BD71828_MASK_LPSR_EN,
			.lpsr_reg = BD71828_REG_BUCK6_SUSP_VOLT,
			.lpsr_mask = BD71828_MASK_BUCK1267_VOLT,
		},
		.reg_inits = buck6_inits,
		.reg_init_amnt = ARRAY_SIZE(buck6_inits),
	},
	{
		.desc = {
			.name = "buck7",
			.of_match = of_match_ptr("BUCK7"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD71828_BUCK7,
			.ops = &bd71828_dvs_buck_ops,
			.type = REGULATOR_VOLTAGE,
			.linear_ranges = bd71828_buck1267_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71828_buck1267_volts),
			.n_voltages = BD71828_BUCK1267_VOLTS,
			.enable_reg = BD71828_REG_BUCK7_EN,
			.enable_mask = BD71828_MASK_RUN_EN,
			.vsel_reg = BD71828_REG_BUCK7_VOLT,
			.vsel_mask = BD71828_MASK_BUCK1267_VOLT,
			.owner = THIS_MODULE,
			.of_parse_cb = buck_set_hw_dvs_levels,
		},
		.dvs = {
			.level_map = ROHM_DVS_LEVEL_RUN | ROHM_DVS_LEVEL_IDLE |
				     ROHM_DVS_LEVEL_SUSPEND |
				     ROHM_DVS_LEVEL_LPSR,
			.run_reg = BD71828_REG_BUCK7_VOLT,
			.run_mask = BD71828_MASK_BUCK1267_VOLT,
			.idle_reg = BD71828_REG_BUCK7_IDLE_VOLT,
			.idle_mask = BD71828_MASK_BUCK1267_VOLT,
			.idle_on_mask = BD71828_MASK_IDLE_EN,
			.suspend_reg = BD71828_REG_BUCK7_SUSP_VOLT,
			.suspend_mask = BD71828_MASK_BUCK1267_VOLT,
			.suspend_on_mask = BD71828_MASK_SUSP_EN,
			.lpsr_on_mask = BD71828_MASK_LPSR_EN,
			.lpsr_reg = BD71828_REG_BUCK7_SUSP_VOLT,
			.lpsr_mask = BD71828_MASK_BUCK1267_VOLT,
		},
		.reg_inits = buck7_inits,
		.reg_init_amnt = ARRAY_SIZE(buck7_inits),
	},
	{
		.desc = {
			.name = "ldo1",
			.of_match = of_match_ptr("LDO1"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD71828_LDO1,
			.ops = &bd71828_ldo_ops,
			.type = REGULATOR_VOLTAGE,
			.linear_ranges = bd71828_ldo_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71828_ldo_volts),
			.n_voltages = BD71828_LDO_VOLTS,
			.enable_reg = BD71828_REG_LDO1_EN,
			.enable_mask = BD71828_MASK_RUN_EN,
			.vsel_reg = BD71828_REG_LDO1_VOLT,
			.vsel_mask = BD71828_MASK_LDO_VOLT,
			.owner = THIS_MODULE,
			.of_parse_cb = buck_set_hw_dvs_levels,
		},
		.dvs = {
			/*
			 * LDO1 only supports single voltage for all states.
			 * voltage can be individually enabled for each state
			 * though => allow setting all states to support
			 * enabling power rail on different states.
			 */
			.level_map = ROHM_DVS_LEVEL_RUN | ROHM_DVS_LEVEL_IDLE |
				     ROHM_DVS_LEVEL_SUSPEND |
				     ROHM_DVS_LEVEL_LPSR,
			.run_reg = BD71828_REG_LDO1_VOLT,
			.idle_reg = BD71828_REG_LDO1_VOLT,
			.suspend_reg = BD71828_REG_LDO1_VOLT,
			.lpsr_reg = BD71828_REG_LDO1_VOLT,
			.run_mask = BD71828_MASK_LDO_VOLT,
			.idle_mask = BD71828_MASK_LDO_VOLT,
			.suspend_mask = BD71828_MASK_LDO_VOLT,
			.lpsr_mask = BD71828_MASK_LDO_VOLT,
			.idle_on_mask = BD71828_MASK_IDLE_EN,
			.suspend_on_mask = BD71828_MASK_SUSP_EN,
			.lpsr_on_mask = BD71828_MASK_LPSR_EN,
		},
	}, {
		.desc = {
			.name = "ldo2",
			.of_match = of_match_ptr("LDO2"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD71828_LDO2,
			.ops = &bd71828_ldo_ops,
			.type = REGULATOR_VOLTAGE,
			.linear_ranges = bd71828_ldo_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71828_ldo_volts),
			.n_voltages = BD71828_LDO_VOLTS,
			.enable_reg = BD71828_REG_LDO2_EN,
			.enable_mask = BD71828_MASK_RUN_EN,
			.vsel_reg = BD71828_REG_LDO2_VOLT,
			.vsel_mask = BD71828_MASK_LDO_VOLT,
			.owner = THIS_MODULE,
			.of_parse_cb = buck_set_hw_dvs_levels,
		},
		.dvs = {
			/*
			 * LDO2 only supports single voltage for all states.
			 * voltage can be individually enabled for each state
			 * though => allow setting all states to support
			 * enabling power rail on different states.
			 */
			.level_map = ROHM_DVS_LEVEL_RUN | ROHM_DVS_LEVEL_IDLE |
				     ROHM_DVS_LEVEL_SUSPEND |
				     ROHM_DVS_LEVEL_LPSR,
			.run_reg = BD71828_REG_LDO2_VOLT,
			.idle_reg = BD71828_REG_LDO2_VOLT,
			.suspend_reg = BD71828_REG_LDO2_VOLT,
			.lpsr_reg = BD71828_REG_LDO2_VOLT,
			.run_mask = BD71828_MASK_LDO_VOLT,
			.idle_mask = BD71828_MASK_LDO_VOLT,
			.suspend_mask = BD71828_MASK_LDO_VOLT,
			.lpsr_mask = BD71828_MASK_LDO_VOLT,
			.idle_on_mask = BD71828_MASK_IDLE_EN,
			.suspend_on_mask = BD71828_MASK_SUSP_EN,
			.lpsr_on_mask = BD71828_MASK_LPSR_EN,
		},
	}, {
		.desc = {
			.name = "ldo3",
			.of_match = of_match_ptr("LDO3"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD71828_LDO3,
			.ops = &bd71828_ldo_ops,
			.type = REGULATOR_VOLTAGE,
			.linear_ranges = bd71828_ldo_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71828_ldo_volts),
			.n_voltages = BD71828_LDO_VOLTS,
			.enable_reg = BD71828_REG_LDO3_EN,
			.enable_mask = BD71828_MASK_RUN_EN,
			.vsel_reg = BD71828_REG_LDO3_VOLT,
			.vsel_mask = BD71828_MASK_LDO_VOLT,
			.owner = THIS_MODULE,
			.of_parse_cb = buck_set_hw_dvs_levels,
		},
		.dvs = {
			/*
			 * LDO3 only supports single voltage for all states.
			 * voltage can be individually enabled for each state
			 * though => allow setting all states to support
			 * enabling power rail on different states.
			 */
			.level_map = ROHM_DVS_LEVEL_RUN | ROHM_DVS_LEVEL_IDLE |
				     ROHM_DVS_LEVEL_SUSPEND |
				     ROHM_DVS_LEVEL_LPSR,
			.run_reg = BD71828_REG_LDO3_VOLT,
			.idle_reg = BD71828_REG_LDO3_VOLT,
			.suspend_reg = BD71828_REG_LDO3_VOLT,
			.lpsr_reg = BD71828_REG_LDO3_VOLT,
			.run_mask = BD71828_MASK_LDO_VOLT,
			.idle_mask = BD71828_MASK_LDO_VOLT,
			.suspend_mask = BD71828_MASK_LDO_VOLT,
			.lpsr_mask = BD71828_MASK_LDO_VOLT,
			.idle_on_mask = BD71828_MASK_IDLE_EN,
			.suspend_on_mask = BD71828_MASK_SUSP_EN,
			.lpsr_on_mask = BD71828_MASK_LPSR_EN,
		},

	}, {
		.desc = {
			.name = "ldo4",
			.of_match = of_match_ptr("LDO4"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD71828_LDO4,
			.ops = &bd71828_ldo_ops,
			.type = REGULATOR_VOLTAGE,
			.linear_ranges = bd71828_ldo_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71828_ldo_volts),
			.n_voltages = BD71828_LDO_VOLTS,
			.enable_reg = BD71828_REG_LDO4_EN,
			.enable_mask = BD71828_MASK_RUN_EN,
			.vsel_reg = BD71828_REG_LDO4_VOLT,
			.vsel_mask = BD71828_MASK_LDO_VOLT,
			.owner = THIS_MODULE,
			.of_parse_cb = buck_set_hw_dvs_levels,
		},
		.dvs = {
			/*
			 * LDO1 only supports single voltage for all states.
			 * voltage can be individually enabled for each state
			 * though => allow setting all states to support
			 * enabling power rail on different states.
			 */
			.level_map = ROHM_DVS_LEVEL_RUN | ROHM_DVS_LEVEL_IDLE |
				     ROHM_DVS_LEVEL_SUSPEND |
				     ROHM_DVS_LEVEL_LPSR,
			.run_reg = BD71828_REG_LDO4_VOLT,
			.idle_reg = BD71828_REG_LDO4_VOLT,
			.suspend_reg = BD71828_REG_LDO4_VOLT,
			.lpsr_reg = BD71828_REG_LDO4_VOLT,
			.run_mask = BD71828_MASK_LDO_VOLT,
			.idle_mask = BD71828_MASK_LDO_VOLT,
			.suspend_mask = BD71828_MASK_LDO_VOLT,
			.lpsr_mask = BD71828_MASK_LDO_VOLT,
			.idle_on_mask = BD71828_MASK_IDLE_EN,
			.suspend_on_mask = BD71828_MASK_SUSP_EN,
			.lpsr_on_mask = BD71828_MASK_LPSR_EN,
		},
	}, {
		.desc = {
			.name = "ldo5",
			.of_match = of_match_ptr("LDO5"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD71828_LDO5,
			.ops = &bd71828_ldo_ops,
			.type = REGULATOR_VOLTAGE,
			.linear_ranges = bd71828_ldo_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71828_ldo_volts),
			.n_voltages = BD71828_LDO_VOLTS,
			.enable_reg = BD71828_REG_LDO5_EN,
			.enable_mask = BD71828_MASK_RUN_EN,
			.vsel_reg = BD71828_REG_LDO5_VOLT,
			.vsel_mask = BD71828_MASK_LDO_VOLT,
			.of_parse_cb = buck_set_hw_dvs_levels,
			.owner = THIS_MODULE,
		},
/* 
		LDO5 is special. It can choose from 2 registers by GPIO.
		This driver supports only configuration where
		BD71828_REG_LDO5_VOLT_L is used.
*/
		.dvs = {
			.level_map = ROHM_DVS_LEVEL_RUN | ROHM_DVS_LEVEL_IDLE |
				     ROHM_DVS_LEVEL_SUSPEND |
				     ROHM_DVS_LEVEL_LPSR,
			.run_reg = BD71828_REG_LDO5_VOLT,
			.idle_reg = BD71828_REG_LDO5_VOLT,
			.suspend_reg = BD71828_REG_LDO5_VOLT,
			.lpsr_reg = BD71828_REG_LDO5_VOLT,
			.run_mask = BD71828_MASK_LDO_VOLT,
			.idle_mask = BD71828_MASK_LDO_VOLT,
			.suspend_mask = BD71828_MASK_LDO_VOLT,
			.lpsr_mask = BD71828_MASK_LDO_VOLT,
			.idle_on_mask = BD71828_MASK_IDLE_EN,
			.suspend_on_mask = BD71828_MASK_SUSP_EN,
			.lpsr_on_mask = BD71828_MASK_LPSR_EN,
		},

	}, {
		.desc = {
			.name = "ldo6",
			.of_match = of_match_ptr("LDO3"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD71828_LDO6,
			.ops = &bd71828_ldo6_ops,
			.type = REGULATOR_VOLTAGE,
			.n_voltages = 1,
			.enable_reg = BD71828_REG_LDO6_EN,
			.enable_mask = BD71828_MASK_RUN_EN,
			.owner = THIS_MODULE,
			/*
			 * LDO6 only supports enable/disable for all states.
			 * Voltage for LDO6 is fixed.
			 */
			.of_parse_cb = ldo6_parse_dt,
		},
	}, {
		.desc = {
			/* SNVS LDO in data-sheet */
			.name = "ldo7",
			.of_match = of_match_ptr("LDO7"),
			.regulators_node = of_match_ptr("regulators"),
			.id = BD71828_LDO_SNVS,
			.ops = &bd71828_ldo_ops,
			.type = REGULATOR_VOLTAGE,
			.linear_ranges = bd71828_ldo_volts,
			.n_linear_ranges = ARRAY_SIZE(bd71828_ldo_volts),
			.n_voltages = BD71828_LDO_VOLTS,
			.enable_reg = BD71828_REG_LDO7_EN,
			.enable_mask = BD71828_MASK_RUN_EN,
			.vsel_reg = BD71828_REG_LDO7_VOLT,
			.vsel_mask = BD71828_MASK_LDO_VOLT,
			.owner = THIS_MODULE,
			.of_parse_cb = buck_set_hw_dvs_levels,
		},
		.dvs = {
			/*
			 * LDO7 only supports single voltage for all states.
			 * voltage can be individually enabled for each state
			 * though => allow setting all states to support
			 * enabling power rail on different states.
			 */
			.level_map = ROHM_DVS_LEVEL_RUN | ROHM_DVS_LEVEL_IDLE |
				     ROHM_DVS_LEVEL_SUSPEND |
				     ROHM_DVS_LEVEL_LPSR,
			.run_reg = BD71828_REG_LDO7_VOLT,
			.idle_reg = BD71828_REG_LDO7_VOLT,
			.suspend_reg = BD71828_REG_LDO7_VOLT,
			.lpsr_reg = BD71828_REG_LDO7_VOLT,
			.run_mask = BD71828_MASK_LDO_VOLT,
			.idle_mask = BD71828_MASK_LDO_VOLT,
			.suspend_mask = BD71828_MASK_LDO_VOLT,
			.lpsr_mask = BD71828_MASK_LDO_VOLT,
			.idle_on_mask = BD71828_MASK_IDLE_EN,
			.suspend_on_mask = BD71828_MASK_SUSP_EN,
			.lpsr_on_mask = BD71828_MASK_LPSR_EN,
		},

	},
};

struct bd71828_gpio_cfg {
	unsigned int gpiobucks;
	struct gpio_desc *gpio1;
	struct gpio_desc *gpio2;
};

static int check_dt_for_gpio_controls(struct device *d,
				      struct bd71828_gpio_cfg *g)
{
	int ret, i;
	struct gpio_desc *tmp;
	struct device_node *np = d->of_node;
	const char *prop = "rohm,dvs_gpio_bucks";
	uint32_t bucks[MAX_GPIO_DVS_BUCKS];

	tmp = gpiod_get_index(d, "rohm,dvs-vsel", 0, GPIOD_OUT_LOW);
	if (IS_ERR(tmp)) {
		ret = PTR_ERR(tmp);
		if (ret == -ENOENT)
			return 0;
		return ret;
	}
	g->gpio1 = tmp;

	tmp = gpiod_get_index(d, "rohm,dvs-vsel", 1, GPIOD_OUT_LOW);
	if (IS_ERR(tmp))
		return PTR_ERR(tmp);

	g->gpio2 = tmp;

	ret = of_property_read_variable_u32_array(np, prop, bucks, 0,
						  ARRAY_SIZE(bucks));

	if (ret < 0) {
		if (ret == -EOVERFLOW)
			return -EINVAL;
	}
	for (i = 0; i < ret; i++)
		g->gpiobucks |= 1 << bucks[i];

	return 0;
}

static void set_buck_gpio_controlled(struct rohm_regmap_dev *bd71828,
				     struct bd71828_regulator_data *rd,
				     struct bd71828_gpio_cfg *g)
{
	switch (rd->desc.id) {
	case BD71828_BUCK1:
		rd->reg_inits = buck1_gpio_inits;
		break;
	case BD71828_BUCK2:
		rd->reg_inits = buck2_gpio_inits;
		break;
	case BD71828_BUCK6:
		rd->reg_inits = buck6_gpio_inits;
		break;
	case BD71828_BUCK7:
		rd->reg_inits = buck7_gpio_inits;
		break;
	default:
		return;
	}
	/*
	 * Disallow setters. Get voltages/enable states based
	 * on current RUN level
	 */
	rd->gpio1 = g->gpio1;
	rd->gpio2 = g->gpio2;
	rd->desc.ops = &dvs_buck_gpio_ops;
	rd->desc.of_parse_cb = buck_set_gpio_hw_dvs_levels;
}

static ssize_t show_runlevel(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	int runlevel;
	struct bd71828_regulator_data *rd = dev_get_drvdata(dev);

	if (!rd || !rd->gpio1)
		return -ENOENT;

	runlevel = bd71828_dvs_gpio_get_run_level(rd);
	if (0 > runlevel)
		return runlevel;

	return sprintf(buf, "0x%x\n", runlevel);
}

static ssize_t set_runlevel(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct bd71828_regulator_data *rd = dev_get_drvdata(dev);
	long val;

	if (kstrtol(buf, 0, &val) != 0)
		return -EINVAL;

	val = bd71828_dvs_gpio_set_run_level(rd, val);
	if (val)
		return val;

	return count;
}

static DEVICE_ATTR(runlevel, 0664, show_runlevel, set_runlevel);

static struct attribute *runlevel_attributes[] = {
	&dev_attr_runlevel.attr,
	NULL
};

static const struct attribute_group bd71828_attr_group = {
	.attrs	= runlevel_attributes,
};

static int bd71828_create_sysfs(struct platform_device *pdev)
{
	return sysfs_create_group(&pdev->dev.kobj, &bd71828_attr_group);
}

static int bd71828_remove_sysfs(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &bd71828_attr_group);
	return 0;
}

static int bd71828_remove(struct platform_device *pdev)
{
	return bd71828_remove_sysfs(pdev);
}

static int bd71828_probe(struct platform_device *pdev)
{
	struct rohm_regmap_dev *bd71828;
	int i, j, ret;
	struct regulator_config config = {
		.dev = pdev->dev.parent,
	};
	struct bd71828_gpio_cfg gcfg = {0};
	struct bd71828_regulator_data *rd;

	bd71828 = dev_get_drvdata(pdev->dev.parent);
	if (!bd71828) {
		dev_err(&pdev->dev, "No MFD driver data\n");
		return -EINVAL;
	}

	ret = check_dt_for_gpio_controls(pdev->dev.parent, &gcfg);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get DVS gpio resources\n");
		return ret;
	}

	/*
	 * Allocate device data to allow controlling more than one PMICs
	 */
	rd = devm_kmalloc_array(&pdev->dev, ARRAY_SIZE(bd71828_rdata),
				sizeof(*rd), GFP_KERNEL);
	if (!rd)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, rd);

	for (i = 0; i < ARRAY_SIZE(bd71828_rdata); i++) {
		/* Use bd71828_rdata as template */
		rd[i] = bd71828_rdata[i];

		if (gcfg.gpiobucks & (1 << i))
			set_buck_gpio_controlled(bd71828, &rd[i], &gcfg);
	}

	config.regmap = bd71828->regmap;

	for (i = 0; i < ARRAY_SIZE(bd71828_rdata); i++) {
		struct regulator_dev *rdev;

		config.driver_data = &rd[i];

		rdev = devm_regulator_register(&pdev->dev,
					       &rd[i].desc, &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev,
				"failed to register %s regulator\n",
				rd[i].desc.name);
			return PTR_ERR(rdev);
		}
		for(j = 0; j < rd[i].reg_init_amnt; j++){
			ret = regmap_update_bits(bd71828->regmap,
						 rd[i].reg_inits[j].reg,
						 rd[i].reg_inits[j].mask,
						 rd[i].reg_inits[j].val);
			if (ret) {
				dev_err(&pdev->dev,
					"regulator %s init failed\n",
					rd[i].desc.name);
				return ret;
			}
		}
	}
	return bd71828_create_sysfs(pdev);
}

static struct platform_driver bd71828_regulator = {
	.driver = {
		.name = "bd71828-pmic"
	},
	.probe = bd71828_probe,
	.remove = bd71828_remove,
};

module_platform_driver(bd71828_regulator);

MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_DESCRIPTION("BD71828 voltage regulator driver");
MODULE_LICENSE("GPL");
