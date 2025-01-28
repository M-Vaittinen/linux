// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2019 ROHM Semiconductors

#include <linux/device.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/leds.h>
#include <linux/mfd/rohm-generic.h>
#include <linux/mfd/rohm-bd71828.h>
#include <linux/mfd/rohm-bd72720.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define BD71828_LED_TO_DATA(l) ((l)->id == ID_GREEN_LED ? \
	container_of((l), struct bd71828_leds, green) : \
	container_of((l), struct bd71828_leds, amber))

enum {
	ID_GREEN_LED,
	ID_AMBER_LED,
	ID_NMBR_OF,
};

struct bd71828_led {
	int id;
	struct led_classdev l;
	u8 force_mask;
};

struct bd71828_leds {
	struct regmap *regmap;
	struct bd71828_led green;
	struct bd71828_led amber;
	u8 force_reg;
};

static int bd71828_led_brightness_set(struct led_classdev *led_cdev,
                                        enum led_brightness value)
{
	struct bd71828_led *l = container_of(led_cdev, struct bd71828_led, l);
	struct bd71828_leds *data;
	unsigned val = BD71828_LED_OFF;

	data = BD71828_LED_TO_DATA(l);	
	if (value != LED_OFF)
		val = BD71828_LED_ON;

	return regmap_update_bits(data->regmap, data->force_reg,
			    l->force_mask, val);
}

/*
 * This function is introduced to drivers/base/property.c later.
 * Drop this internal copy of it when updating the kernel
 */
static int fwnode_property_match_property_string(const struct fwnode_handle *fwnode,
	const char *propname, const char * const *array, size_t n)
{
	const char *string;
	int ret;

	ret = fwnode_property_read_string(fwnode, propname, &string);
	if (ret)
		return ret;

	ret = match_string(array, n, string);
	if (ret < 0)
		ret = -ENOENT;

	return ret;
}

static int bd71828_led_probe(struct platform_device *pdev)
{
	enum rohm_chip_type chip = platform_get_device_id(pdev)->driver_data;
	struct fwnode_handle *np, *child;;
	struct led_init_data init_data = {};
	struct bd71828_leds *l;
	struct bd71828_led *g, *a;
	struct regmap *r;
	int ret, found = 0;

	r = dev_get_regmap(pdev->dev.parent, NULL);
	if (!r)
		return dev_err_probe(&pdev->dev, -ENODEV, "No regmap");

	np = device_get_named_child_node(pdev->dev.parent, "leds");
        if (!np)
                return -ENODEV;

	l = devm_kzalloc(&pdev->dev, sizeof(*l), GFP_KERNEL);
	if (!l)
		return -ENOMEM;

	l->regmap = r;
	a = &l->amber;
	g = &l->green;
	a->id = ID_AMBER_LED;
	g->id = ID_GREEN_LED;
	a->force_mask = BD71828_MASK_LED_AMBER;
	g->force_mask = BD71828_MASK_LED_GREEN;

	a->l.brightness_set_blocking = bd71828_led_brightness_set;
	g->l.brightness_set_blocking = bd71828_led_brightness_set;

	switch (chip) {
		case ROHM_CHIP_TYPE_BD71828:
			l->force_reg = BD71828_REG_LED_CTRL;
			break;
		case ROHM_CHIP_TYPE_BD72720:
			l->force_reg = BD72720_REG_LED_CTRL;
			break;
		default:
			dev_err(&pdev->dev, "Unknown IC");
			return -EINVAL;
	}

	fwnode_for_each_available_child_node(np, child) {
		const char * const compat[] = { "bd71828-grnled", "bd71828-ambled", "bd72720-grnled", "bd72720-ambled" };

		ret = fwnode_property_match_property_string(child,
				"rohm,led-compatible", compat,
				ARRAY_SIZE(compat));
		if (ret < 0)
			continue;

		init_data.fwnode = child;
		if (ret == 0 || ret == 2) {
			ret = devm_led_classdev_register_ext(&pdev->dev, &g->l, &init_data);
			if (ret)
				return ret;
			found++;

		} else if (ret == 1 || ret == 3) {
			ret = devm_led_classdev_register_ext(&pdev->dev, &a->l, &init_data);
			if (ret)
				return ret;
			found++;
		}
	}

	if (!found)
		return -ENODEV;

	return 0;
}

static const struct platform_device_id bd71828_led_id[] = {
	{ "bd71828-led", ROHM_CHIP_TYPE_BD71828 },
	{ "bd72720-led", ROHM_CHIP_TYPE_BD72720 },
	{ },
};
MODULE_DEVICE_TABLE(platform, bd71828_led_id);

static struct platform_driver bd71828_led_driver = {
        .driver = {
                .name  = "bd71828-led",
        },
        .probe  = bd71828_led_probe,
	.id_table = bd71828_led_id,
};

module_platform_driver(bd71828_led_driver);

MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_DESCRIPTION("ROHM BD71828 LED driver");
MODULE_LICENSE("GPL");
