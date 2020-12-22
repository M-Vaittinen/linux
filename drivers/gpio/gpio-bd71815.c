// SPDX-License-Identifier: GPL-2.0
/*
 * Support to GPOs on ROHM BD71815
 */

#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/irq.h>
/* For the BD71815 register definitions */
#include <linux/mfd/rohm-bd71815.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

struct bd71815_gpio {
	struct gpio_chip chip;
	struct device *dev;
	struct regmap *regmap;
	/*
	 * Sigh. The BD71815 and BD71817 were originally designed to support two
	 * GPO pins. At some point it was noticed the second GPO pin which is
	 * the E5 pin located at the center of IC is hard to use on PCB (due to
	 * the location). It was decided to not promote this second GPO and pin
	 * is marked as GND on the data-sheet. The functionality is still there
	 * though! I guess driving GPO connected to ground is a bad idea. Thus
	 * we do not support it by default. OTOH - the original driver written
	 * by colleagues at Embest did support controlling this second GPO. It
	 * is thus possible this is used in some of the products.
	 *
	 * This driver does not by default support configuring this second GPO
	 * but allows using it by providing the DT property
	 * "rohm,enable-hidden-gpo".
	 */
	bool e5_pin_is_gpo;
};

static int bd71815gpo_get(struct gpio_chip *chip, unsigned int offset)
{
	struct bd71815_gpio *bd71815 = gpiochip_get_data(chip);
	int ret = 0;
	int val;

	ret = regmap_read(bd71815->regmap, BD71815_REG_GPO, &val);
	if (ret)
		return ret;

	return (val >> offset) & 1;
}

static void bd71815gpo_set(struct gpio_chip *chip, unsigned int offset,
			   int value)
{
	struct bd71815_gpio *bd71815 = gpiochip_get_data(chip);
	int ret, bit;

	if (!bd71815->e5_pin_is_gpo && offset)
		return;

	bit = BIT(offset);

	if (value)
		ret = regmap_set_bits(bd71815->regmap, BD71815_REG_GPO, bit);
	else
		ret = regmap_clear_bits(bd71815->regmap, BD71815_REG_GPO, bit);

	if (ret)
		dev_warn(bd71815->dev, "failed to toggle GPO\n");
}

static int bd71815_gpio_set_config(struct gpio_chip *chip, unsigned int offset,
				   unsigned long config)
{
	struct bd71815_gpio *bdgpio = gpiochip_get_data(chip);

	if (!bdgpio->e5_pin_is_gpo && offset)
		return -EOPNOTSUPP;

	switch (pinconf_to_config_param(config)) {
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		return regmap_update_bits(bdgpio->regmap,
					  BD71815_REG_GPO,
					  BD71815_GPIO_DRIVE_MASK << offset,
					  BD71815_GPIO_OPEN_DRAIN << offset);
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		return regmap_update_bits(bdgpio->regmap,
					  BD71815_REG_GPO,
					  BD71815_GPIO_DRIVE_MASK << offset,
					  BD71815_GPIO_CMOS << offset);
	default:
		break;
	}
	return -EOPNOTSUPP;
}

/* BD71815 GPIO is actually GPO */
static int bd71815gpo_direction_get(struct gpio_chip *gc, unsigned int offset)
{
	return GPIO_LINE_DIRECTION_OUT;
}

/* Template for GPIO chip */
static const struct gpio_chip bd71815gpo_chip = {
	.label			= "bd71815",
	.owner			= THIS_MODULE,
	.get			= bd71815gpo_get,
	.get_direction		= bd71815gpo_direction_get,
	.set			= bd71815gpo_set,
	.set_config		= bd71815_gpio_set_config,
	.can_sleep		= 1,
};

static int gpo_bd71815_probe(struct platform_device *pdev)
{
	int ret;
	struct bd71815_gpio *g;
	struct device *dev;
	struct device *parent;

	/*
	 * Bind devm lifetime to this platform device => use dev for devm.
	 * also the prints should originate from this device.
	 */
	dev = &pdev->dev;
	/* The device-tree and regmap come from MFD => use parent for that */
	parent = dev->parent;

	g = devm_kzalloc(dev, sizeof(*g), GFP_KERNEL);
	if (!g)
		return -ENOMEM;

	g->e5_pin_is_gpo = of_property_read_bool(parent->of_node,
						 "rohm,enable-hidden-gpo");
	g->chip = bd71815gpo_chip;
	g->chip.base = -1;

	if (g->e5_pin_is_gpo)
		g->chip.ngpio = 2;
	else
		g->chip.ngpio = 1;

	g->chip.parent = parent;
	g->chip.of_node = parent->of_node;
	g->regmap = dev_get_regmap(parent, NULL);
	g->dev = dev;

	ret = devm_gpiochip_add_data(dev, &g->chip, g);
	if (ret < 0) {
		dev_err(dev, "could not register gpiochip, %d\n", ret);
		return ret;
	}

	return ret;
}
static const struct platform_device_id bd7181x_gpo_id[] = {
	{ "bd71815-gpo" },
	{ },
};
MODULE_DEVICE_TABLE(platform, bd7181x_gpo_id);

static struct platform_driver gpo_bd71815_driver = {
	.driver = {
		.name	= "bd71815-gpo",
		.owner	= THIS_MODULE,
	},
	.probe		= gpo_bd71815_probe,
	.id_table	= bd7181x_gpo_id,
};

module_platform_driver(gpo_bd71815_driver);

/* Note:  this hardware lives inside an I2C-based multi-function device. */
MODULE_ALIAS("platform:bd71815-gpo");

MODULE_AUTHOR("Peter Yang <yanglsh@embest-tech.com>");
MODULE_DESCRIPTION("GPO interface for BD71815");
MODULE_LICENSE("GPL");
