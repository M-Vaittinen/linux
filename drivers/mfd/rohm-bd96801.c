// SPDX-License-Identifier: GPL-2.0-or-later
//
// Copyright (C) 2020 ROHM Semiconductors
//
// ROHM BD96801 PMIC driver

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mfd/rohm-bd96801.h>
#include <linux/mfd/rohm-generic.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/types.h>

static struct mfd_cell bd96801_mfd_cells[] = {
	{ .name = "bd96801-pmic", },
};


static const struct regmap_range pmic_status_range[] = {
	{
		.range_min = BD96801_REG_PWR_CTRL, /* EN pin status */
		.range_max = BD96801_REG_PWR_CTRL,
	}, {
		.range_min = BD96801_REG_WD_FEED,
		.range_max = BD96801_REG_WD_FAILCOUNT,
	}, {
		.range_min = BD96801_REG_WD_ASK,
		.range_max = BD96801_REG_WD_ASK,
	}, {
		.range_min = BD96801_REG_WD_STATUS,
		.range_max = BD96801_REG_WD_STATUS,
	}, {
		.range_min = BD96801_REG_PMIC_STATE,
		.range_max = BD96801_REG_INT_LDO7,
	},
};

static const struct regmap_access_table volatile_regs = {
	.yes_ranges = pmic_status_range,
	.n_yes_ranges = ARRAY_SIZE(pmic_status_range),
};
#if 0
/* TBD: ROHM BD96801 irqs */
enum {
	BD718XX_INT_STBY_REQ,
	BD718XX_INT_ON_REQ,
	BD718XX_INT_WDOG,
	BD718XX_INT_PWRBTN,
	BD718XX_INT_PWRBTN_L,
	BD718XX_INT_PWRBTN_S,
	BD718XX_INT_SWRST
};

/* TBD: BD96801 irq registers and masks */
static const struct regmap_irq bd96801_irqs[] = {
	REGMAP_IRQ_REG(BD718XX_INT_SWRST, 0, BD718XX_INT_SWRST_MASK),
	REGMAP_IRQ_REG(BD718XX_INT_PWRBTN_S, 0, BD718XX_INT_PWRBTN_S_MASK),
	REGMAP_IRQ_REG(BD718XX_INT_PWRBTN_L, 0, BD718XX_INT_PWRBTN_L_MASK),
	REGMAP_IRQ_REG(BD718XX_INT_PWRBTN, 0, BD718XX_INT_PWRBTN_MASK),
	REGMAP_IRQ_REG(BD718XX_INT_WDOG, 0, BD718XX_INT_WDOG_MASK),
	REGMAP_IRQ_REG(BD718XX_INT_ON_REQ, 0, BD718XX_INT_ON_REQ_MASK),
	REGMAP_IRQ_REG(BD718XX_INT_STBY_REQ, 0, BD718XX_INT_STBY_REQ_MASK),
};

static struct regmap_irq_chip bd96801_irq_chip = {
	.name = "bd96801-irq",
	.irqs = bd96801_irqs,
	.num_irqs = ARRAY_SIZE(bd96801_irqs),
	.num_regs = 1,
	.irq_reg_stride = 1,
	.status_base = BD96801_REG_IRQ,
	.mask_base = BD96801_REG_MIRQ,
	.ack_base = BD96801_REG_IRQ,
	.init_ack_masked = true,
	.mask_invert = false,
};
#endif

static const struct regmap_config bd96801_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_table = &volatile_regs,
	.max_register = BD96801_MAX_REGISTER - 1,
	.cache_type = REGCACHE_RBTREE,
};


static int bd96801_i2c_probe(struct i2c_client *i2c)
{
	struct rohm_regmap_dev *bd96801;
	int ret;
//	struct regmap_irq_chip_data *irq_data;

	if (!i2c->irq) {
		dev_err(&i2c->dev, "No IRQ configured\n");
		return -EINVAL;
	}

	bd96801 = devm_kzalloc(&i2c->dev, sizeof(*bd96801), GFP_KERNEL);
	if (!bd96801)
		return -ENOMEM;

	bd96801->dev = &i2c->dev;
	dev_set_drvdata(&i2c->dev, bd96801);

	bd96801->regmap = devm_regmap_init_i2c(i2c, &bd96801_regmap_config);
	if (IS_ERR(bd96801->regmap)) {
		dev_err(&i2c->dev, "regmap initialization failed\n");
		return PTR_ERR(bd96801->regmap);
	}
/*
	ret = devm_regmap_add_irq_chip(&i2c->dev, bd96801->regmap,
				       i2c->irq, IRQF_ONESHOT, 0,
				       &bd96801_irq_chip, &irq_data);
	if (ret) {
		dev_err(&i2c->dev, "Failed to add irq_chip\n");
		return ret;
	}
*/
	ret = devm_mfd_add_devices(bd96801->dev, PLATFORM_DEVID_AUTO,
				   bd96801_mfd_cells,
				   ARRAY_SIZE(bd96801_mfd_cells), NULL, 0,
				   /* regmap_irq_get_domain(irq_data) */ NULL );
	if (ret)
		dev_err(&i2c->dev, "Failed to create subdevices\n");

	return ret;
}

static const struct of_device_id bd96801_of_match[] = {
	{
		.compatible = "rohm,bd96801",
	},
	{ }
};
MODULE_DEVICE_TABLE(of, bd96801_of_match);

static struct i2c_driver bd96801_i2c_driver = {
	.driver = {
		.name = "rohm-bd96801",
		.of_match_table = bd96801_of_match,
	},
	.probe_new = bd96801_i2c_probe,
};

static int __init bd96801_i2c_init(void)
{
	return i2c_add_driver(&bd96801_i2c_driver);
}

/* Initialise early so consumer devices can complete system boot */
subsys_initcall(bd96801_i2c_init);

static void __exit bd96801_i2c_exit(void)
{
	i2c_del_driver(&bd96801_i2c_driver);
}
module_exit(bd96801_i2c_exit);

MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_DESCRIPTION("ROHM BD96801 Power Management IC driver");
MODULE_LICENSE("GPL");
