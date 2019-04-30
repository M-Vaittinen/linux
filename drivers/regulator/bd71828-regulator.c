// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2019 ROHM Semiconductors
// bd71828-regulator.c ROHM BD71828GW-DS1 regulator driver
//

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
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

static const struct regulator_linear_range bd71828_buck6_volts[] = {
	REGULATOR_LINEAR_RANGE(),
	REGULATOR_LINEAR_RANGE(),

static const struct regulator_ops bd70528_buck_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.set_ramp_delay = bd70528_set_ramp_delay,
};

static const struct regulator_desc bd71828_desc[] = {
	{
		.name = "buck1",
		.of_match = of_match_ptr("BUCK1"),
		.regulators_node = of_match_ptr("regulators"),
		.id = BD71828_BUCK1,
		.ops = &bd71828_buck_ops,
		.type = REGULATOR_VOLTAGE,
		.linear_ranges = bd71828_buck1267_volts,
		.n_linear_ranges = ARRAY_SIZE(bd71828_buck1267_volts),
// 		Why this would be needed when linear ranges are in use??
//		.n_voltages = BD71828_BUCK1267_VOLTS,
		.enable_reg = BD71828_REG_BUCK1_EN,
		.enable_mask = BD71828_MASK_RUN_EN,
		.vsel_reg = BD71828_REG_BUCK1_VOLT,
		.vsel_mask = BD71828_MASK_BUCK1267_VOLT,
		.owner = THIS_MODULE,
	},
	{
		.name = "buck2",
		.of_match = of_match_ptr("BUCK2"),
		.regulators_node = of_match_ptr("regulators"),
		.id = BD71828_BUCK2,
		.ops = &bd71828_buck_ops,
		.type = REGULATOR_VOLTAGE,
		.linear_ranges = bd71828_buck1267_volts,
		.n_linear_ranges = ARRAY_SIZE(bd71828_buck1267_volts),
		.n_voltages = BD71828_BUCK1267_VOLTS,
		.enable_reg = BD71828_REG_BUCK2_EN,
		.enable_mask = BD71828_MASK_RUN_EN,
		.vsel_reg = BD71828_REG_BUCK2_VOLT,
		.vsel_mask = BD71828_MASK_BUCK1267_VOLT,
		.owner = THIS_MODULE,
	},
	{
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
	},
	{
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
	},
	{
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
	},
	{
		.name = "buck6",
		.of_match = of_match_ptr("BUCK6"),
		.regulators_node = of_match_ptr("regulators"),
		.id = BD71828_BUCK6,
		.ops = &bd71828_buck_ops,
		.type = REGULATOR_VOLTAGE,
		.linear_ranges = bd71828_buck1267_volts,
		.n_linear_ranges = ARRAY_SIZE(bd71828_buck1267_volts),
		.n_voltages = BD71828_BUCK1267_VOLTS,
		.enable_reg = BD71828_REG_BUCK6_EN,
		.enable_mask = BD71828_MASK_RUN_EN,
		.vsel_reg = BD71828_REG_BUCK6_VOLT,
		.vsel_mask = BD71828_MASK_BUCK1267_VOLT,
		.owner = THIS_MODULE,
	},
	{
		.name = "buck7",
		.of_match = of_match_ptr("BUCK7"),
		.regulators_node = of_match_ptr("regulators"),
		.id = BD71828_BUCK7,
		.ops = &bd71828_buck_ops,
		.type = REGULATOR_VOLTAGE,
		.linear_ranges = bd71828_buck1267_volts,
		.n_linear_ranges = ARRAY_SIZE(bd71828_buck1267_volts),
		.n_voltages = BD71828_BUCK1267_VOLTS,
		.enable_reg = BD71828_REG_BUCK7_EN,
		.enable_mask = BD71828_MASK_RUN_EN,
		.vsel_reg = BD71828_REG_BUCK7_VOLT,
		.vsel_mask = BD71828_MASK_BUCK1267_VOLT,
		.owner = THIS_MODULE,
	},

//Bucks done
/// TODO Bucks  5, ...
	{
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
	},
	{
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
	},
	{
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
	},
	{
		.name = "ldo_led1",
		.of_match = of_match_ptr("LDO_LED1"),
		.regulators_node = of_match_ptr("regulators"),
		.id = BD71828_LED1,
		.ops = &bd71828_led_ops,
		.type = REGULATOR_VOLTAGE,
		.volt_table = &led_volts[0],
		.n_voltages = ARRAY_SIZE(led_volts),
		.enable_reg = BD71828_REG_LED_EN,
		.enable_mask = BD71828_MASK_LED1_EN,
		.vsel_reg = BD71828_REG_LED_VOLT,
		.vsel_mask = BD71828_MASK_LED1_VOLT,
		.owner = THIS_MODULE,
	},
	{
		.name = "ldo_led2",
		.of_match = of_match_ptr("LDO_LED2"),
		.regulators_node = of_match_ptr("regulators"),
		.id = BD71828_LED2,
		.ops = &bd71828_led_ops,
		.type = REGULATOR_VOLTAGE,
		.volt_table = &led_volts[0],
		.n_voltages = ARRAY_SIZE(led_volts),
		.enable_reg = BD71828_REG_LED_EN,
		.enable_mask = BD71828_MASK_LED2_EN,
		.vsel_reg = BD71828_REG_LED_VOLT,
		.vsel_mask = BD71828_MASK_LED2_VOLT,
		.owner = THIS_MODULE,
	},
};

static int bd71828_probe(struct platform_device *pdev)
{
	struct rohm_regmap_dev *bd71828;
	int i;
	struct regulator_config config = {
		.dev = pdev->dev.parent,
	};

	bd71828 = dev_get_drvdata(pdev->dev.parent);
	if (!bd71828) {
		dev_err(&pdev->dev, "No MFD driver data\n");
		return -EINVAL;
	}

	config.regmap = bd71828->regmap;

	for (i = 0; i < ARRAY_SIZE(bd71828_desc); i++) {
		struct regulator_dev *rdev;

		rdev = devm_regulator_register(&pdev->dev, &bd71828_desc[i],
					       &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev,
				"failed to register %s regulator\n",
				bd71828_desc[i].name);
			return PTR_ERR(rdev);
		}
	}
	return 0;
}

static struct platform_driver bd71828_regulator = {
	.driver = {
		.name = "bd71828-pmic"
	},
	.probe = bd71828_probe,
};

module_platform_driver(bd71828_regulator);

MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_DESCRIPTION("BD71828 voltage regulator driver");
MODULE_LICENSE("GPL");
