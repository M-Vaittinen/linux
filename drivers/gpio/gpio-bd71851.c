// SPDX-License-Identifier: GPL-2.0
/*
 * Support to GPIOs on ROHM BD71851
 * Copyright 2024 ROHM Semiconductors.
 * Author: Matti Vaittinen <mazziesaccount@gmail.com>
 */

#include <linux/gpio/driver.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/mfd/rohm-bd71851.h>

#define BD71851_GPIO_MAX_PINS 4
/*
 * The BD71851 has several "one time programmable" (OTP) configurations which
 * can be set at manufacturing phase. A some of these options allow using
 * individual pins as GPI or GPO (not both at the same time). The OTP
 * configuration can't be read at run-time, so drivers rely on device-tree to
 * advertise the OTP programmed in manufacturing.
 *
 * The pins which can be used as GPIO are:
 * GPIO1, CLKOUT, FAULT_B, EXTEN_OUT.
 *
 * The OTP options 2 and 3 state for all the pins:
 *  - OTP2: GPI (also IRQ source)
 *  - OTP3: GPO (NOTE: This is actually 2 different OTP options. Either a
 *    register controllable output or a power-sequence controlled output.
 *    The "gpo" referred here means only the register controllable output.
 *    The datasheet refers to this as: "<pin> output is controlled by
 *    GPIO<N>_OUT or power on/off sequencer to control external VRs. ON/OFF
 *    sequence timing is configurable."
 *
 * The data-sheet further says that the GPI/GPO is not a default OTP
 * configuration for any of the pins. Hence the GPIO driver defaults to a pin
 * not being a GPI or GPO, but requires the pin to be explicitly marked as a
 * GPI or GPO in the device-tree.
 *
 * DT properties:
 * "rohm,pin-gpio1", "rohm,pin-clkout", "rohm,pin-fault-b", "rohm,pin-exten"
 * can be set to one of the values "gpi" or "gpo" to enable them to be used as
 * GPIO.
 */

enum bd71851_gpio_state {
	BD71851_PIN_UNKNOWN,
	BD71851_PIN_GPI,
	BD71851_PIN_GPO,
};

struct bd71851_gpio_pin_cfg {
	enum bd71851_gpio_state state;
	int mask; /* GPIO_OUT and INT_SRC have same bit offsets for GPIO */
};

struct bd71851_gpio {
	/* chip.parent points the MFD which provides DT node and regmap */
	struct gpio_chip chip;
	struct bd71851_gpio_pin_cfg pin[BD71851_GPIO_MAX_PINS];
	int num_pins;
	/* dev points to the platform device for devm and prints */
	struct device *dev;
	struct regmap *regmap;
};

static int bd71851_gpio_get_pins(struct bd71851_gpio *g)
{
	const char *properties[] = { "rohm,pin-gpio1", "rohm,pin-clkout",
				     "rohm,pin-fault-b", "rohm,pin-exten" };
	const char *val;
	int i, ret;

	for ( i = 0; i < ARRAY_SIZE(properties); i++) {
		ret = fwnode_property_read_string(dev_fwnode(g->dev->parent),
						  properties[i], &val);

		if (ret) {
			if (ret == -EINVAL)
				continue;

			return dev_err_probe(g->dev, ret,
					"pin %d (%s), bad configuration\n", i,
					properties[i]);
		}

		if (strcmp(val, "gpi") == 0) {
			g->pin[g->num_pins].state = BD71851_PIN_GPI;
			g->pin[g->num_pins].mask = BIT(i);
			g->num_pins++;
		} else if (strcmp(val, "gpo") == 0) {
			g->pin[g->num_pins].state = BD71851_PIN_GPO;
			g->pin[g->num_pins].mask = BIT(i);
			g->num_pins++;
		}
	}

	return 0;
}

static int bd71851gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	struct bd71851_gpio *bdgpio = gpiochip_get_data(chip);
	struct bd71851_gpio_pin_cfg *pin = &bdgpio->pin[offset];
	int ret, val;

	/* Only pins configured as GPI via OTP can have their status read */
	if (pin->state != BD71851_PIN_GPI) {
		dev_dbg(bdgpio->dev, "pin %d (%x) not input. State %d\n",
			offset, pin->mask, pin->state);
		return -EINVAL;
	}

	ret = regmap_read(bdgpio->regmap, BD71851_REG_INT_5_SRC, &val);
	if (ret)
		return ret;

	return val & pin->mask;
}

static void bd71851gpo_set(struct gpio_chip *chip, unsigned int offset,
			   int value)
{
	struct bd71851_gpio *bdgpio = gpiochip_get_data(chip);
	struct bd71851_gpio_pin_cfg *pin = &bdgpio->pin[offset];
	int ret;

	if (pin->state != BD71851_PIN_GPO) {
		dev_dbg(bdgpio->dev, "pin %d (%d) not output. State %d\n",
			offset, pin->mask, pin->state);
		return;
	}

	if (value)
		ret = regmap_set_bits(bdgpio->regmap, BD71851_REG_GPO_OUT, pin->mask);
	else
		ret = regmap_clear_bits(bdgpio->regmap, BD71851_REG_GPO_OUT, pin->mask);
	if (ret)
		dev_warn(bdgpio->dev, "failed to toggle GPO\n");
}

static int bd71851gpio_direction_get(struct gpio_chip *chip,
				    unsigned int offset)
{
	struct bd71851_gpio *bdgpio = gpiochip_get_data(chip);

	if (bdgpio->pin[offset].state == BD71851_PIN_GPO)
		return GPIO_LINE_DIRECTION_OUT;

	return GPIO_LINE_DIRECTION_IN;
}

/*
 * Template for GPIO chip. The BD71851 GPO supports both CMOS and open drain
 * configurations. The default however depends on the OTP. The runtime config
 * can be done via undocumented test registers - but at the moment there is no
 * support for this.
 */
static const struct gpio_chip bd71851gpio_chip = {
	.label			= "bd71851",
	.owner			= THIS_MODULE,
	.get			= bd71851gpio_get,
	.get_direction		= bd71851gpio_direction_get,
	.set			= bd71851gpo_set,
	.can_sleep		= true,
};

static int gpo_bd71851_probe(struct platform_device *pdev)
{
	struct bd71851_gpio *g;
	struct device *parent, *dev;
	int ret;

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

	g->chip = bd71851gpio_chip;


	g->chip.base = -1;
	g->chip.parent = parent;
	g->regmap = dev_get_regmap(parent, NULL);
	g->dev = dev;

	ret = bd71851_gpio_get_pins(g);
	if (ret)
		return ret;

	if (!g->num_pins) {
		/*
		 * The BD71851 may or may not have pins allocated for GPIO
		 * depending on the OTP used at manufacturing. Free the memory
		 * and go out if there is no pins as then we have nothing to do
		 */
		dev_dbg(dev, "no GPIO pins\n");
		devm_kfree(dev, g);
		return 0;
	}
	g->chip.ngpio = g->num_pins;

	return devm_gpiochip_add_data(dev, &g->chip, g);
}

static const struct platform_device_id bd71851_gpio_id[] = {
	{ "bd71851-gpio" },
	{ },
};
MODULE_DEVICE_TABLE(platform, bd71851_gpio_id);

static struct platform_driver gpo_bd71851_driver = {
	.driver = {
		.name = "bd71851-gpio",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = gpo_bd71851_probe,
	.id_table = bd71851_gpio_id,
};
module_platform_driver(gpo_bd71851_driver);

MODULE_AUTHOR("Matti Vaittinen <mazziesaccount@gmail.com>");
MODULE_DESCRIPTION("GPIO interface for BD71851");
MODULE_LICENSE("GPL");
