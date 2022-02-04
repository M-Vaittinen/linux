// SPDX-License-Identifier: GPL-2.0-only
//
// Copyright (C) 2022 ROHM Semiconductors
//
// ROHM/KIONIX KX132 accelerometer driver

#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/gpio.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_gpio.h>
#endif
#include <linux/regulator/consumer.h>

#define KX132_ID		0x3d

#define KX132_REG_MAN_ID	0x0
#define KX132_REG_XADP_L	0x02
#define KX132_REG_ZOUT_H	0x0d
#define KX132_REG_COTR		0x12
#define KX132_REG_WHO		0x13
#define KX132_REG_TSCP		0x14
#define KX132_REG_INT_REL	0x1a
#define KX132_REG_BUF_STAT1	0x60
#define KX132_REG_BUF_STAT2	0x61
#define KX132_REG_BUF_READ	0x63
#define KX132_REG_SELF_TEST	0x5d
#define KX132_REG_BUF_CLEAR	0x62

#define KX132_MAX_REGISTER	0x76

/* TODO: Check if this is Ok */
#define KX132_INPUT_DEV_EVENTS_NUM 120

static const struct regmap_range kx132_volatile_ranges[] = {
	{
		.range_min = KX132_REG_XADP_L,
		.range_max = KX132_REG_ZOUT_H,
	}, {
		.range_min = KX132_REG_COTR,
		.range_max = KX132_REG_COTR,
	}, {
		.range_min = KX132_REG_TSCP,
		.range_max = KX132_REG_INT_REL,
	}, {
		.range_min = KX132_REG_SELF_TEST,
		.range_max = KX132_REG_SELF_TEST,
	}, {
		.range_min = KX132_REG_BUF_CLEAR,
		.range_max = KX132_REG_BUF_CLEAR,
	},
};

static const struct regmap_access_table kx132_volatile_regs = {
	.yes_ranges = &kx132_volatile_ranges[0],
	.n_yes_ranges = ARRAY_SIZE(kx132_volatile_ranges),
};

static const struct regmap_range kx132_read_only_ranges[] = {
	{
		.range_min = KX132_REG_MAN_ID,
		.range_max = KX132_REG_ZOUT_H,
	}, {
		.range_min = KX132_REG_COTR,
		.range_max = KX132_REG_INT_REL,
	}, {
		.range_min = KX132_REG_BUF_STAT1,
		.range_max = KX132_REG_BUF_STAT2,
	}, {
		.range_min = KX132_REG_BUF_READ,
		.range_max = KX132_REG_BUF_READ,
	},
};

static const struct regmap_access_table kx132_ro_regs = {
	.no_ranges = &kx132_read_only_ranges[0],
	.n_no_ranges = ARRAY_SIZE(kx132_read_only_ranges),
};

static const struct regmap_range kx132_write_only_ranges[] = {
	{
		.range_min = KX132_REG_SELF_TEST,
		.range_max = KX132_REG_SELF_TEST,
	}, {
		.range_min = KX132_REG_BUF_CLEAR,
		.range_max = KX132_REG_BUF_CLEAR,
	},
};

static const struct regmap_access_table kx132_wo_regs = {
	.no_ranges = &kx132_write_only_ranges[0],
	.n_no_ranges = ARRAY_SIZE(kx132_write_only_ranges),
};

static const struct regmap_config kx132_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.volatile_table = &kx132_volatile_regs,
	.wr_table = &kx132_wo_regs,
	.rd_table = &kx132_ro_regs,
	.max_register = KX132_MAX_REGISTER,
	.cache_type = REGCACHE_RBTREE,
};

struct kx132 {
	struct device *dev;
	struct regmap *regmap;
	u32 x_map;
	u32 y_map;
	u32 z_map;
	u32 x_negate;
	u32 y_negate;
	u32 z_negate;
	u32 g_range;
};

enum {
	KX132_IRQ_DRDY,
	KX132_IRQ_WMI,
};

/*
 * 0 => 2
 * 1 => 4
 * 2 => 8
 * 3 => 16
 *
 * G = 2 >> SEL
 */
static int kx132_g2regval(int g)
{
	int i;

	for (i = 0; i < 3 && (2 << i) < g; i++)
		;

	return i;
}

static int kx132_parse_dt(struct fwnode_handle *fw, struct kx132 *kx)
{
	int ret, i;
	u32 *val[] = { &kx->x_map, &kx->y_map, &kx->z_map, &kx->x_negate,
		       &kx->y_negate, &kx->z_negate, &kx->g_range };

	/* TODO: Could 'negate' ones be boolean properties? */
	static const char * const prop[] = {"kionix,x-map", "kionix,y-map",
		"kionix,z-map", "kionix,x-negate", "kionix,y-negate",
		"kionix,z-negate", "kionix,g-range-gravity" };

	/* mandatory dt parameters */
	for (i = 0; i < ARRAY_SIZE(prop); i++) {
		ret = fwnode_property_read_u32(fw, prop[i], val[i]);
		if (ret)
			dev_warn(kx->dev, "Failed to read '%s'\n", prop[i]);
	}

	/* TODO: Device-tree should represent real-world values rather than
	 * arbitrary register settings. Find a proper units for G values and
	 * use those from the DT. It'd be good to stay on integers so think of
	 * units that provide reasonable accuray.
	 */
	//sdata->pdata.g_range = kx132_map_value_to_grange((u8) temp_val);

	/* optional dt parameters i.e. use poll if gpio-int not found */
/*
	sdata->pdata.gpio_int1 = of_get_named_gpio_flags(dev->of_node,
		"kionix,gpio-int1", 0, NULL);

	sdata->pdata.gpio_int2 = of_get_named_gpio_flags(dev->of_node,
		"kionix,gpio-int2", 0, NULL);

	sdata->pdata.use_drdy_int = of_property_read_bool(dev->of_node,
		"kionix,use-drdy-int");
*/
	return 0;
}


static int kx132_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct fwnode_handle *fw;
	struct kx132 *kx;
	struct input_dev *id;
	int ident, ret;

	if (!i2c->irq) {
		dev_err(dev, "No IRQ configured\n");
		return -EINVAL;
	}

	fw = dev_fwnode(dev);
	if (!fw)
		return -ENODEV;

	kx = devm_kzalloc(dev, sizeof(*kx), GFP_KERNEL);
	if (!kx)
		return -ENOMEM;

	kx->regmap = devm_regmap_init_i2c(i2c, &kx132_regmap);
	if (IS_ERR(kx->regmap)) {
		dev_err(dev, "Failed to initialize Regmap\n");

		return PTR_ERR(kx->regmap);
	}

	ret = regmap_read(kx->regmap, KX132_REG_WHO, &ident);
	if (ret) {
		dev_err(dev, "Failed to access sensor\n");

		return ret;
	}
	if (ident != KX132_ID) {
		dev_err(dev, "unsupported device 0x%x\n", ident);

		return -EINVAL;
	}

	ret = kx132_parse_dt(fw, kx);
	if (ret)
		return ret;
/*
 * TODO: Do.
	if (fwnode_property_read_bool(fw, "kionix,i2c-pin-secondary"))
		ret = route_irq_optional(kx);

	if (ret)
		return ret;
*/
	id = devm_input_allocate_device(dev);
	if (!id)
		return -ENOMEM;

	id->name = "kx132-accel";
	id->id.bustype = BUS_I2C;
	id->id.vendor = (int)"KION";
	id->dev.parent = dev;
	set_bit(EV_ABS, id->evbit);
	input_set_abs_params(id, ABS_X, INT_MIN, INT_MAX,0,0);
	input_set_abs_params(id, ABS_Y, INT_MIN, INT_MAX,0,0);
	input_set_abs_params(id, ABS_Z, INT_MIN, INT_MAX,0,0);

	input_set_events_per_packet(id, KX132_INPUT_DEV_EVENTS_NUM);

	input_set_drvdata(id, kx);

	return 0;
}

static const struct of_device_id kx132_of_match[] = {
	{ .compatible = "kionix,kx132", },
	{ },
};
MODULE_DEVICE_TABLE(of, kx132_of_match);
/*
static const struct dev_pm_ops kx132_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(kx132_suspend, kx132_resume)
	SET_RUNTIME_PM_OPS(kx132_runtime_suspend, kx132_runtime_resume, NULL)
};
*/
static struct i2c_driver kx132_driver = {
	.driver = {
			.name  = "kx132",
//			.pm    = &kx132_pm_ops,
			.of_match_table = kx132_of_match,
		  },
	.probe_new    = kx132_probe,
};

module_i2c_driver(kx132_driver);

MODULE_DESCRIPTION("ROHM/Kionix kx132 accelerometer drove");
MODULE_AUTHOR("Heikki Haikola <heikki.haikola@fi.rohmeurope.com>");
MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_LICENSE("GPL");

