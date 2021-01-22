// SPDX-License-Identifier: GPL-2.0
// Copyright (C) 2020 ROHM Semiconductors
// ROHM BD9576MUF/BD9573MUF regulator driver

//#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/mfd/rohm-bd957x.h>
#include <linux/mfd/rohm-generic.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/of_regulator.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>

#define BD957X_VOUTS1_VOLT	3300000
#define BD957X_VOUTS4_BASE_VOLT	1030000
#define BD957X_VOUTS34_NUM_VOLT	32

static int vout1_volt_table[] = {5000000, 4900000, 4800000, 4700000, 4600000,
				 4500000, 4500000, 4500000, 5000000, 5100000,
				 5200000, 5300000, 5400000, 5500000, 5500000,
				 5500000};

static int vout2_volt_table[] = {1800000, 1780000, 1760000, 1740000, 1720000,
				 1700000, 1680000, 1660000, 1800000, 1820000,
				 1840000, 1860000, 1880000, 1900000, 1920000,
				 1940000};

static int voutl1_volt_table[] = {2500000, 2540000, 2580000, 2620000, 2660000,
				  2700000, 2740000, 2780000, 2500000, 2460000,
				  2420000, 2380000, 2340000, 2300000, 2260000,
				  2220000};

struct bd957x_regulator_data {
	struct regulator_desc desc;
	int base_voltage;
	struct regulator_dev *rdev;
	int regulator_err; /* Error flag set from IRQ */
};

#define BD9576_NUM_REGULATORS 6
struct bd957x_data {
	struct bd957x_regulator_data regulator_data[BD9576_NUM_REGULATORS];
	struct regmap *regmap;
	struct delayed_work therm_irq_suppress;
	struct delayed_work ovd_irq_suppress;
	struct delayed_work uvd_irq_suppress;
	unsigned int therm_irq;
	unsigned int ovd_irq;
	unsigned int uvd_irq;
	spinlock_t err_lock;
	int regulator_global_err;
};

static int bd957x_vout34_list_voltage(struct regulator_dev *rdev,
				      unsigned int selector)
{
	const struct regulator_desc *desc = rdev->desc;
	int multiplier = selector & desc->vsel_mask & 0x7f;
	int tune;

	/* VOUT3 and 4 has 10mV step */
	tune = multiplier * 10000;

	if (!(selector & 0x80))
		return desc->fixed_uV - tune;

	return desc->fixed_uV + tune;
}

static int bd957x_list_voltage(struct regulator_dev *rdev,
			       unsigned int selector)
{
	const struct regulator_desc *desc = rdev->desc;
	int index = selector & desc->vsel_mask & 0x7f;

	if (!(selector & 0x80))
		index += desc->n_voltages/2;

	if (index >= desc->n_voltages)
		return -EINVAL;

	return desc->volt_table[index];
}

static int bd9576_get_error_flags(struct regulator_dev *rdev,
				  unsigned int *flags)
{
	struct bd957x_data *d;
	struct bd957x_regulator_data *r;

	r = container_of(rdev->desc, struct bd957x_regulator_data, desc);
	d = rdev_get_drvdata(rdev);

	spin_lock(&d->err_lock);
	*flags = d->regulator_global_err | r->regulator_err;
	spin_unlock(&d->err_lock);

	return 0;
}

static const struct regulator_ops bd957x_vout34_ops = {
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = bd957x_vout34_list_voltage,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.get_error_flags = bd9576_get_error_flags,
};

static const struct regulator_ops bd957X_vouts1_regulator_ops = {
	.is_enabled = regulator_is_enabled_regmap,
	.get_error_flags = bd9576_get_error_flags,
};

static const struct regulator_ops bd957x_ops = {
	.is_enabled = regulator_is_enabled_regmap,
	.list_voltage = bd957x_list_voltage,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.get_error_flags = bd9576_get_error_flags,
};

static struct bd957x_data bd957x_regulators = {
	.regulator_data = {
		{
			.desc = {
				.name = "VD50",
				.of_match = of_match_ptr("regulator-vd50"),
				.regulators_node = of_match_ptr("regulators"),
				.id = BD957X_VD50,
				.type = REGULATOR_VOLTAGE,
				.ops = &bd957x_ops,
				.volt_table = &vout1_volt_table[0],
				.n_voltages = ARRAY_SIZE(vout1_volt_table),
				.vsel_reg = BD957X_REG_VOUT1_TUNE,
				.vsel_mask = BD957X_MASK_VOUT1_TUNE,
				.enable_reg = BD957X_REG_POW_TRIGGER1,
				.enable_mask = BD957X_REGULATOR_EN_MASK,
				.enable_val = BD957X_REGULATOR_DIS_VAL,
				.enable_is_inverted = true,
				.owner = THIS_MODULE,
			},
		},
		{
			.desc = {
				.name = "VD18",
				.of_match = of_match_ptr("regulator-vd18"),
				.regulators_node = of_match_ptr("regulators"),
				.id = BD957X_VD18,
				.type = REGULATOR_VOLTAGE,
				.ops = &bd957x_ops,
				.volt_table = &vout2_volt_table[0],
				.n_voltages = ARRAY_SIZE(vout2_volt_table),
				.vsel_reg = BD957X_REG_VOUT2_TUNE,
				.vsel_mask = BD957X_MASK_VOUT2_TUNE,
				.enable_reg = BD957X_REG_POW_TRIGGER2,
				.enable_mask = BD957X_REGULATOR_EN_MASK,
				.enable_val = BD957X_REGULATOR_DIS_VAL,
				.enable_is_inverted = true,
				.owner = THIS_MODULE,
			},
		},
		{
			.desc = {
				.name = "VDDDR",
				.of_match = of_match_ptr("regulator-vdddr"),
				.regulators_node = of_match_ptr("regulators"),
				.id = BD957X_VDDDR,
				.ops = &bd957x_vout34_ops,
				.type = REGULATOR_VOLTAGE,
				.n_voltages = BD957X_VOUTS34_NUM_VOLT,
				.vsel_reg = BD957X_REG_VOUT3_TUNE,
				.vsel_mask = BD957X_MASK_VOUT3_TUNE,
				.enable_reg = BD957X_REG_POW_TRIGGER3,
				.enable_mask = BD957X_REGULATOR_EN_MASK,
				.enable_val = BD957X_REGULATOR_DIS_VAL,
				.enable_is_inverted = true,
				.owner = THIS_MODULE,
			},
		},
		{
			.desc = {
				.name = "VD10",
				.of_match = of_match_ptr("regulator-vd10"),
				.regulators_node = of_match_ptr("regulators"),
				.id = BD957X_VD10,
				.ops = &bd957x_vout34_ops,
				.type = REGULATOR_VOLTAGE,
				.fixed_uV = BD957X_VOUTS4_BASE_VOLT,
				.n_voltages = BD957X_VOUTS34_NUM_VOLT,
				.vsel_reg = BD957X_REG_VOUT4_TUNE,
				.vsel_mask = BD957X_MASK_VOUT4_TUNE,
				.enable_reg = BD957X_REG_POW_TRIGGER4,
				.enable_mask = BD957X_REGULATOR_EN_MASK,
				.enable_val = BD957X_REGULATOR_DIS_VAL,
				.enable_is_inverted = true,
				.owner = THIS_MODULE,
			},
		},
		{
			.desc = {
				.name = "VOUTL1",
				.of_match = of_match_ptr("regulator-voutl1"),
				.regulators_node = of_match_ptr("regulators"),
				.id = BD957X_VOUTL1,
				.ops = &bd957x_ops,
				.type = REGULATOR_VOLTAGE,
				.volt_table = &voutl1_volt_table[0],
				.n_voltages = ARRAY_SIZE(voutl1_volt_table),
				.vsel_reg = BD957X_REG_VOUTL1_TUNE,
				.vsel_mask = BD957X_MASK_VOUTL1_TUNE,
				.enable_reg = BD957X_REG_POW_TRIGGERL1,
				.enable_mask = BD957X_REGULATOR_EN_MASK,
				.enable_val = BD957X_REGULATOR_DIS_VAL,
				.enable_is_inverted = true,
				.owner = THIS_MODULE,
			},
		},
		{
			.desc = {
				.name = "VOUTS1",
				.of_match = of_match_ptr("regulator-vouts1"),
				.regulators_node = of_match_ptr("regulators"),
				.id = BD957X_VOUTS1,
				.ops = &bd957X_vouts1_regulator_ops,
				.type = REGULATOR_VOLTAGE,
				.n_voltages = 1,
				.fixed_uV = BD957X_VOUTS1_VOLT,
				.enable_reg = BD957X_REG_POW_TRIGGERS1,
				.enable_mask = BD957X_REGULATOR_EN_MASK,
				.enable_val = BD957X_REGULATOR_DIS_VAL,
				.enable_is_inverted = true,
				.owner = THIS_MODULE,
			},
		},
	},
};

#define BD9576_THERM_IRQ_MASK_TW	BIT(5)
#define BD9576_xVD_IRQ_MASK_VOUTL1	BIT(5)
#define BD9576_UVD_IRQ_MASK_VOUTS1_OCW	BIT(6)
void delayed_enable_irq(struct delayed_work *w, int irq)
{
	/* keep the "main" IRQ masked for 1 sec */
	disable_irq_nosync(irq);
	schedule_delayed_work(w, msecs_to_jiffies(IRQS_SILENT_MS));
}

static void bd9576_vd_err(struct bd957x_data *d,
			  struct bd957x_regulator_data *r, int err)
{
	if (!r->rdev)
		return;

	spin_lock(&d->err_lock);
	r->regulator_err |= err;
	spin_unlock(&d->err_lock);
	regulator_notifier_call_chain(r->rdev, err, NULL);
}

static irqreturn_t bd9576_irq_uvd(int irq, void *data)
{
	struct bd957x_data *d = (struct bd957x_data *)data;
	int val, ret, i;

	ret = regmap_read(d->regmap, BD957X_REG_INT_UVD_STAT, &val);
	if (ret)
		return IRQ_NONE;

	/* bits [0] ... [3] represent xVD source VOUT1 ... VOUT4 */
	for (i = 0; i < 4; i++)
		if ((1 << i) & val)
			bd9576_vd_err(d, &d->regulator_data[i],
				      REGULATOR_ERROR_UNDER_VOLTAGE);

	if (val & BD9576_xVD_IRQ_MASK_VOUTL1)
		bd9576_vd_err(d, &d->regulator_data[BD957X_VOUTL1],
			      REGULATOR_ERROR_UNDER_VOLTAGE);

	if (val & BD9576_UVD_IRQ_MASK_VOUTS1_OCW)
		bd9576_vd_err(d, &d->regulator_data[BD957X_VOUTS1],
			      REGULATOR_ERROR_OVER_CURRENT);

	/* Clear the sub-IRQ status */
	regmap_update_bits(d->regmap, BD957X_REG_INT_UVD_STAT,
			   UVD_IRQ_VALID_MASK, val);
	delayed_enable_irq(&d->uvd_irq_suppress, irq);

	return IRQ_HANDLED;
}

static irqreturn_t bd9576_irq_ovd(int irq, void *data)
{
	struct bd957x_data *d = (struct bd957x_data *)data;
	int val, ret, i;

	ret = regmap_read(d->regmap, BD957X_REG_INT_OVD_STAT, &val);
	if (ret)
		return IRQ_NONE;

	if (!(val & OVD_IRQ_VALID_MASK))
		return IRQ_NONE;

	/* bits [0] ... [3] represent xVD source VOUT1 ... VOUT4 */
	for (i = 0; i < 4; i++)
		if ((1 << i) & val)
			bd9576_vd_err(d, &d->regulator_data[i],
				      REGULATOR_ERROR_REGULATION_OUT);

	if (val & BD9576_xVD_IRQ_MASK_VOUTL1)
		bd9576_vd_err(d, &d->regulator_data[BD957X_VOUTL1],
			      REGULATOR_ERROR_REGULATION_OUT);

	/* Clear the sub-IRQ status */
	regmap_update_bits(d->regmap, BD957X_REG_INT_OVD_STAT,
			   OVD_IRQ_VALID_MASK, val);
	delayed_enable_irq(&d->ovd_irq_suppress, irq);

	return IRQ_HANDLED;
}

static irqreturn_t bd9576_irq_thermal(int irq, void *data)
{
	struct bd957x_data *d = (struct bd957x_data *)data;
	int val, ret;

	ret = regmap_read(d->regmap, BD957X_REG_INT_THERM_STAT, &val);
	if (ret)
		return IRQ_NONE;

	if (!(val & UVD_IRQ_VALID_MASK))
		return IRQ_NONE;

	if (val & BD9576_THERM_IRQ_MASK_TW) {
		int i;

		d->regulator_global_err = REGULATOR_ERROR_OVER_TEMP;

		for (i = 0; i < BD9576_NUM_REGULATORS; i++) {
			struct regulator_dev *rdev;

			rdev = d->regulator_data[i].rdev;
			if (rdev)
				regulator_notifier_call_chain(rdev,
					REGULATOR_EVENT_OVER_TEMP, NULL);
		}

		/* Clear the sub-IRQ status */
		regmap_update_bits(d->regmap, BD957X_REG_INT_THERM_STAT,
				   BD9576_THERM_IRQ_MASK_TW,
				   BD9576_THERM_IRQ_MASK_TW);
	} else {
		return IRQ_NONE;
	}
	delayed_enable_irq(&d->therm_irq_suppress, irq);

	return IRQ_HANDLED;
}

/*
 * BD9576 does not have a register adverticing the current status of errors.
 * We just clean the errors when unmasking. If problem is still "on" the IRQ
 * will re-trigger immediately => we send new notification + toggle the error
 * flag back "on" in IRQ handler.
 */
static void therm_irq_work(struct work_struct *w)
{
	struct bd957x_data *d;

	d = container_of(w, struct bd957x_data, therm_irq_suppress.work);
	d->regulator_global_err = 0;
	enable_irq(d->therm_irq);
}

static void ovd_irq_work(struct work_struct *w)
{
	struct bd957x_data *d;
	int i, clear = REGULATOR_ERROR_REGULATION_OUT;

	d = container_of(w, struct bd957x_data, ovd_irq_suppress.work);
	spin_lock(&d->err_lock);
	for (i = 0; i < BD9576_NUM_REGULATORS; i++)
		d->regulator_data[i].regulator_err &= clear;
	spin_unlock(&d->err_lock);
	enable_irq(d->ovd_irq);
}

static void uvd_irq_work(struct work_struct *w)
{
	int clear, i;
	struct bd957x_data *d;

	d = container_of(w, struct bd957x_data, uvd_irq_suppress.work);
	clear = (REGULATOR_ERROR_OVER_CURRENT | REGULATOR_ERROR_UNDER_VOLTAGE);

	spin_lock(&d->err_lock);
	for (i = 0; i < BD9576_NUM_REGULATORS; i++)
		d->regulator_data[i].regulator_err &= clear;
	spin_unlock(&d->err_lock);
	enable_irq(d->uvd_irq);
}

static int bd9576_get_irqs(struct platform_device *pdev, struct regmap *regmap,
			   struct bd957x_data *data)
{
	int irq, ret;

	spin_lock_init(&data->err_lock);
	irq = platform_get_irq_byname(pdev, "bd9576-temp");
	if (irq > 0) {
		data->therm_irq = irq;
		INIT_DELAYED_WORK(&data->therm_irq_suppress, therm_irq_work);
		ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
						&bd9576_irq_thermal,
						IRQF_ONESHOT, "bd9576-temp",
						data);
		if (ret)
			return ret;
		/*
		 * Enable therm IRQ from sub IRQ mask
		 */
		ret = regmap_update_bits(regmap, BD957X_REG_INT_THERM_MASK,
					 BD9576_THERM_IRQ_MASK_TW, 0);
		if (ret)
			return ret;
	}
	irq = platform_get_irq_byname(pdev, "bd9576-ovd");
	if (irq > 0) {
		data->ovd_irq = irq;
		INIT_DELAYED_WORK(&data->ovd_irq_suppress, ovd_irq_work);
		ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
						&bd9576_irq_ovd,
						IRQF_ONESHOT, "bd9576-ovd",
						data);
		if (ret)
			return ret;

	}
	irq = platform_get_irq_byname(pdev, "bd9576-uvd");
	if (irq > 0) {
		data->uvd_irq = irq;
		INIT_DELAYED_WORK(&data->uvd_irq_suppress, uvd_irq_work);
		ret = devm_request_threaded_irq(&pdev->dev, irq, NULL,
						&bd9576_irq_uvd,
						IRQF_ONESHOT, "bd9576-uvd",
						data);
		if (ret)
			return ret;
	}

	return 0;
}

static int bd957x_probe(struct platform_device *pdev)
{
	struct regmap *regmap;
	struct regulator_config config = { 0 };
	int i, err;
	bool vout_mode, ddr_sel;
	struct bd957x_data *ic_data;
	unsigned int num_reg_data;
	enum rohm_chip_type chip = platform_get_device_id(pdev)->driver_data;

	num_reg_data = ARRAY_SIZE(bd957x_regulators.regulator_data);

	ic_data = &bd957x_regulators;

	regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!regmap) {
		dev_err(&pdev->dev, "No regmap\n");
		return -EINVAL;
	}
	ic_data->regmap = regmap;
	vout_mode = of_property_read_bool(pdev->dev.parent->of_node,
					 "rohm,vout1-en-low");
	if (vout_mode) {
		struct gpio_desc *en;

		dev_dbg(&pdev->dev, "GPIO controlled mode\n");

		/* VOUT1 enable state judged by VOUT1_EN pin */
		/* See if we have GPIO defined */
		en = devm_gpiod_get_from_of_node(&pdev->dev,
						 pdev->dev.parent->of_node,
						 "rohm,vout1-en-gpios", 0,
						 GPIOD_OUT_LOW, "vout1-en");
		if (!IS_ERR(en)) {
			/* VOUT1_OPS gpio ctrl */
			/*
			 * Regulator core prioritizes the ena_gpio over
			 * enable/disable/is_enabled callbacks so no need to
			 * clear them. We can still use same ops
			 */
			config.ena_gpiod = en;
		} else {
			/*
			 * In theory it is possible someone wants to set
			 * vout1-en LOW during OTP loading and set VOUT1 to be
			 * controlled by GPIO - but control the GPIO from some
			 * where else than this driver. For that to work we
			 * should unset the is_enabled callback here.
			 *
			 * I believe such case where rohm,vout1-en-low is set
			 * and vout1-en-gpios is not is likely to be a
			 * misconfiguration. So let's just err out for now.
			 */
			dev_err(&pdev->dev,
				"Failed to get VOUT1 control GPIO\n");
			return PTR_ERR(en);
		}
	}

	/*
	 * If more than one PMIC needs to be controlled by same processor then
	 * allocate the regulator data array here and use bd9576_regulators as
	 * template. At the moment I see no such use-case so I spare some
	 * bytes and use bd9576_regulators directly for non-constant configs
	 * like DDR voltage selection.
	 */
	platform_set_drvdata(pdev, ic_data);
	ddr_sel =  of_property_read_bool(pdev->dev.parent->of_node,
					 "rohm,ddr-sel-low");
	if (ddr_sel)
		ic_data->regulator_data[2].desc.fixed_uV = 1350000;
	else
		ic_data->regulator_data[2].desc.fixed_uV = 1500000;

	switch (chip) {
	case ROHM_CHIP_TYPE_BD9576:
		/*
		 * And also here we pass pointer to static bd9576_regulators
		 * for IRQs to use. If we one day allocate driver data - then
		 * this needs to be changed too.
		 */
		err = bd9576_get_irqs(pdev, regmap, ic_data);
		if (err)
			goto err_out;
		dev_dbg(&pdev->dev, "Found BD9576MUF\n");
		break;
	case ROHM_CHIP_TYPE_BD9573:
		dev_dbg(&pdev->dev, "Found BD9573MUF\n");
		break;
	default:
		dev_err(&pdev->dev, "Unsupported chip type\n");
		err = -EINVAL;
		goto err_out;
	}

	config.dev = pdev->dev.parent;
	config.regmap = regmap;
	config.driver_data = ic_data;

	for (i = 0; i < num_reg_data; i++) {

		struct bd957x_regulator_data *r = &ic_data->regulator_data[i];
		const struct regulator_desc *desc = &r->desc;

		r->rdev = devm_regulator_register(&pdev->dev, desc,
							   &config);
		if (IS_ERR(r->rdev)) {
			dev_err(&pdev->dev,
				"failed to register %s regulator\n",
				desc->name);
			err = PTR_ERR(r->rdev);
			goto err_out;
		}
		/*
		 * Clear the VOUT1 GPIO setting - rest of the regulators do not
		 * support GPIO control
		 */
		config.ena_gpiod = NULL;
	}

err_out:
	if (err) {
		if (ic_data->therm_irq)
			cancel_delayed_work_sync(&ic_data->therm_irq_suppress);
		if (ic_data->ovd_irq)
			cancel_delayed_work_sync(&ic_data->ovd_irq_suppress);
		if (ic_data->uvd_irq)
			cancel_delayed_work_sync(&ic_data->uvd_irq_suppress);
	}
	return err;
}

static int bd957x_remove(struct platform_device *pdev)
{
	struct bd957x_data *ic_data = platform_get_drvdata(pdev);

	if (ic_data->therm_irq)
		cancel_delayed_work_sync(&ic_data->therm_irq_suppress);
	if (ic_data->ovd_irq)
		cancel_delayed_work_sync(&ic_data->ovd_irq_suppress);
	if (ic_data->uvd_irq)
		cancel_delayed_work_sync(&ic_data->uvd_irq_suppress);

	return 0;
}

static const struct platform_device_id bd957x_pmic_id[] = {
	{ "bd9573-pmic", ROHM_CHIP_TYPE_BD9573 },
	{ "bd9576-pmic", ROHM_CHIP_TYPE_BD9576 },
	{ },
};
MODULE_DEVICE_TABLE(platform, bd957x_pmic_id);

static struct platform_driver bd957x_regulator = {
	.driver = {
		.name = "bd957x-pmic",
	},
	.probe = bd957x_probe,
	.remove = bd957x_remove,
	.id_table = bd957x_pmic_id,
};

module_platform_driver(bd957x_regulator);

MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_DESCRIPTION("ROHM BD9576/BD9573 voltage regulator driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:bd957x-pmic");
