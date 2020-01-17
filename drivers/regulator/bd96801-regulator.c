// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2020 ROHM Semiconductors
// bd96801-regulator.c ROHM BD96801 regulator driver

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mfd/rohm-generic.h>
//#include <linux/mfd/rohm-bd96801.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>

enum {
	BD96801_BUCK1,
	BD96801_BUCK2,
	BD96801_BUCK3,
	BD96801_BUCK4,
	BD96801_LDO5,
	BD96801_LDO6,
	BD96801_LDO7,
	BD96801_REGULATOR_AMOUNT,
};

#define BD96801_REG_ENABLE 0xb
#define BD96801_BUCK1_EN_MASK 0x1
#define BD96801_BUCK2_EN_MASK 0x2
#define BD96801_BUCK3_EN_MASK 0x4
#define BD96801_BUCK4_EN_MASK 0x8
#define BD96801_LDO5_EN_MASK 0x10
#define BD96801_LDO6_EN_MASK 0x20
#define BD96801_LDO7_EN_MASK 0x40

/*
 * TODO: Check what is the difference between *_INT_VOUT and *_VOUT registers
 * Maybe this needs to be changed
 */
#define BD96801_BUCK1_VSEL_REG 0x21
#define BD96801_BUCK2_VSEL_REG 0x22
#define BD96801_BUCK3_VSEL_REG 0x23
#define BD96801_BUCK4_VSEL_REG 0x24
#define BD96801_LDO5_VSEL_REG 0x25
#define BD96801_LDO6_VSEL_REG 0x26
#define BD96801_LDO7_VSEL_REG 0x27
#define BD96801_BUCK_VSEL_MASK 0xff
#define BD96801_LDO_VSEL_MASK 0x7f

#define BD96801_LOCK_REG 0x4
#define BD96801_UNLOCK 0x9d
#define BD96801_LOCK 0x0

#define BD96801_RAMP_1MVUS 0
#define BD96801_RAMP_5MVUS 1
#define BD96801_RAMP_10MVUS 2
#define BD96801_RAMP_20MVUS 3
#define BD96801_MASK_RAMP_DELAY 0xc0
#define BD96801_RAMP_BASE_REG 0x28

#define BD96801_BUCK_VOLTS 256
#define BD96801_LDO_VOLTS 128

static const struct regulator_linear_range bd96801_buck_volts[] = {
	REGULATOR_LINEAR_RANGE(500000, 0x00, 0xc8, 5000),
	REGULATOR_LINEAR_RANGE(1550000, 0xc9, 0xec, 50000),
	REGULATOR_LINEAR_RANGE(3300000, 0xed, 0xff, 0),
};

static const struct regulator_linear_range bd96801_ldo_volts[] = {
	REGULATOR_LINEAR_RANGE(300000, 0x00, 0x78, 25000),
	REGULATOR_LINEAR_RANGE(3300000, 0x79, 0x7f, 0),
};

static int bd96801_set_ramp_delay(struct regulator_dev *rdev, int ramp_delay)
{
	unsigned int val;
	unsigned int reg = BD96801_RAMP_BASE_REG + rdev->desc->id;

	switch (ramp_delay) {
	case 1 ... 1000:
		val = BD96801_RAMP_1MVUS;
		break;
	case 1001 ... 5000:
		val = BD96801_RAMP_5MVUS;
		break;
	case 5001 ... 10000:
		val = BD96801_RAMP_10MVUS;
		break;
	case 10001 ... 20000:
		val = BD96801_RAMP_20MVUS;
		break;
	default:
		val = BD96801_RAMP_10MVUS;
		dev_err(&rdev->dev,
			"ramp_delay: %d not supported, setting 20mV/uS",
			 ramp_delay);
	}

	return regmap_update_bits(rdev->regmap, reg, BD96801_MASK_RAMP_DELAY,
				  val << (ffs(BD96801_MASK_RAMP_DELAY) - 1));
}

static const struct regulator_ops bd96801_buck_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
	.set_ramp_delay = bd96801_set_ramp_delay,
};

static const struct regulator_ops bd96801_ldo_ops = {
	.enable = regulator_enable_regmap,
	.disable = regulator_disable_regmap,
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = regulator_list_voltage_linear_range,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_voltage_time_sel = regulator_set_voltage_time_sel,
};

static const struct regulator_desc bd96801_desc[] = {
	{
		.name = "buck1",
		.of_match = of_match_ptr("BUCK1"),
		.regulators_node = of_match_ptr("regulators"),
		.id = BD96801_BUCK1,
		.ops = &bd96801_buck_ops,
		.type = REGULATOR_VOLTAGE,
		.linear_ranges = bd96801_buck_volts,
		.n_linear_ranges = ARRAY_SIZE(bd96801_buck_volts),
		.n_voltages = BD96801_BUCK_VOLTS,
		.enable_reg = BD96801_REG_ENABLE,
		.enable_mask = BD96801_BUCK1_EN_MASK,
		.vsel_reg = BD96801_BUCK1_VSEL_REG,
		.vsel_mask = BD96801_BUCK_VSEL_MASK,
		.owner = THIS_MODULE,
	},
	{
		.name = "buck2",
		.of_match = of_match_ptr("BUCK2"),
		.regulators_node = of_match_ptr("regulators"),
		.id = BD96801_BUCK2,
		.ops = &bd96801_buck_ops,
		.type = REGULATOR_VOLTAGE,
		.linear_ranges = bd96801_buck_volts,
		.n_linear_ranges = ARRAY_SIZE(bd96801_buck_volts),
		.n_voltages = BD96801_BUCK_VOLTS,
		.enable_reg = BD96801_REG_ENABLE,
		.enable_mask = BD96801_BUCK2_EN_MASK,
		.vsel_reg = BD96801_BUCK2_VSEL_REG,
		.vsel_mask = BD96801_BUCK_VSEL_MASK,
		.owner = THIS_MODULE,
	},
	{
		.name = "buck3",
		.of_match = of_match_ptr("BUCK3"),
		.regulators_node = of_match_ptr("regulators"),
		.id = BD96801_BUCK3,
		.ops = &bd96801_buck_ops,
		.type = REGULATOR_VOLTAGE,
		.linear_ranges = bd96801_buck_volts,
		.n_linear_ranges = ARRAY_SIZE(bd96801_buck_volts),
		.n_voltages = BD96801_BUCK_VOLTS,
		.enable_reg = BD96801_REG_ENABLE,
		.enable_mask = BD96801_BUCK3_EN_MASK,
		.vsel_reg = BD96801_BUCK3_VSEL_REG,
		.vsel_mask = BD96801_BUCK_VSEL_MASK,
		.owner = THIS_MODULE,
	},{
		.name = "buck4",
		.of_match = of_match_ptr("BUCK4"),
		.regulators_node = of_match_ptr("regulators"),
		.id = BD96801_BUCK4,
		.ops = &bd96801_buck_ops,
		.type = REGULATOR_VOLTAGE,
		.linear_ranges = bd96801_buck_volts,
		.n_linear_ranges = ARRAY_SIZE(bd96801_buck_volts),
		.n_voltages = BD96801_BUCK_VOLTS,
		.enable_reg = BD96801_REG_ENABLE,
		.enable_mask = BD96801_BUCK4_EN_MASK,
		.vsel_reg = BD96801_BUCK4_VSEL_REG,
		.vsel_mask = BD96801_BUCK_VSEL_MASK,
		.owner = THIS_MODULE,
	},
	{
		.name = "ldo5",
		.of_match = of_match_ptr("LDO5"),
		.regulators_node = of_match_ptr("regulators"),
		.id = BD96801_LDO5,
		.ops = &bd96801_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.linear_ranges = bd96801_ldo_volts,
		.n_linear_ranges = ARRAY_SIZE(bd96801_ldo_volts),
		.n_voltages = BD96801_LDO_VOLTS,
		.enable_reg = BD96801_REG_ENABLE,
		.enable_mask = BD96801_LDO5_EN_MASK,
		.vsel_reg = BD96801_LDO5_VSEL_REG,
		.vsel_mask = BD96801_LDO_VSEL_MASK,
		.owner = THIS_MODULE,
	},
	{
		.name = "ldo6",
		.of_match = of_match_ptr("LDO6"),
		.regulators_node = of_match_ptr("regulators"),
		.id = BD96801_LDO6,
		.ops = &bd96801_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.linear_ranges = bd96801_ldo_volts,
		.n_linear_ranges = ARRAY_SIZE(bd96801_ldo_volts),
		.n_voltages = BD96801_LDO_VOLTS,
		.enable_reg = BD96801_REG_ENABLE,
		.enable_mask = BD96801_LDO6_EN_MASK,
		.vsel_reg = BD96801_LDO6_VSEL_REG,
		.vsel_mask = BD96801_LDO_VSEL_MASK,
		.owner = THIS_MODULE,
	},
	{
		.name = "ldo7",
		.of_match = of_match_ptr("LDO7"),
		.regulators_node = of_match_ptr("regulators"),
		.id = BD96801_LDO7,
		.ops = &bd96801_ldo_ops,
		.type = REGULATOR_VOLTAGE,
		.linear_ranges = bd96801_ldo_volts,
		.n_linear_ranges = ARRAY_SIZE(bd96801_ldo_volts),
		.n_voltages = BD96801_LDO_VOLTS,
		.enable_reg = BD96801_REG_ENABLE,
		.enable_mask = BD96801_LDO7_EN_MASK,
		.vsel_reg = BD96801_LDO7_VSEL_REG,
		.vsel_mask = BD96801_LDO_VSEL_MASK,
		.owner = THIS_MODULE,
	},
};

static int bd96801_probe(struct platform_device *pdev)
{
	struct rohm_regmap_dev *parent;
	int i, ret;
	struct regulator_config config = {
		.dev = pdev->dev.parent,
	};

	parent = dev_get_drvdata(pdev->dev.parent);
	if (!parent) {
		dev_err(&pdev->dev, "No MFD driver data\n");
		return -EINVAL;
	}

	config.regmap = parent->regmap;

	ret = regmap_write(parent->regmap, BD96801_LOCK_REG, BD96801_UNLOCK);
	if (ret) {
		dev_err(&pdev->dev, "Can't unlock PMIC\n");
		return ret;
	}

	for (i = 0; i < ARRAY_SIZE(bd96801_desc); i++) {
		struct regulator_dev *rdev;

		rdev = devm_regulator_register(&pdev->dev, &bd96801_desc[i],
					       &config);
		if (IS_ERR(rdev)) {
			dev_err(&pdev->dev,
				"failed to register %s regulator\n",
				bd96801_desc[i].name);
			return PTR_ERR(rdev);
		}
	}
	return 0;
}

static struct platform_driver bd96801_regulator = {
	.driver = {
		.name = "bd96801-pmic"
	},
	.probe = bd96801_probe,
};

module_platform_driver(bd96801_regulator);

MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_DESCRIPTION("BD96801 voltage regulator driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bd96801-pmic");
