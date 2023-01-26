// SPDX-License-Identifier: GPL-2.0-only
/*
 * BU27010 ROHM Colour Sensor
 *
 * Copyright (c) 2023, ROHM Semiconductor.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#define BU27010_REG_POWER		0x3e
#define BU27010_MASK_POWER		NIT(0)

#define BU27010_REG_RESET		0x3f
#define BU27010_MASK_RESET		NIT(0)

#define BU27010_REG_SYSTEM_CONTROL	0x40
#define BU27010_MASK_SW_RESET		BIT(7)
#define BU27010_MASK_PART_ID		GENMASK(5, 0)

#define BU27010_REG_MODE_CONTROL1	0x41
#define BU27010_MASK_RGB_SEL		GENMASK(7, 6)
#define BU27010_MASK_MEAS_MODE		GENMASK(5, 4)
#define BU27010_MASK_RGB_GAIN		GENMASK(3, 0)

#define BU27010_REG_MODE_CONTROL2	0x42
#define BU27010_MASK_DATA3_GAIN		GENMASK(7, 6)
#define BU27010_MASK_DATA2_GAIN		GENMASK(5, 4)
#define BU27010_MASK_DATA1_GAIN		GENMASK(3, 2)
#define BU27010_MASK_DATA0_GAIN		GENMASK(1, 0)

#define BU27010_REG_MODE_CONTROL3	0x43
#define BU27010_MASK_FLC_MODE		BIT(7)
#define BU27010_MASK_FLC_GAIN		GENMASK(4, 0)

#define BU27010_REG_MODE_CONTROL4	0x44
#define BU27010_MASK_WTM_TH		GENMASK(3, 2)
#define BU27010_MASK_INT_SEL		GENMASK(1, 0)

#define BU27010_REG_MODE_CONTROL5	0x45
#define BU27010_MASK_RGB_VALID		BIT(7)
#define BU27010_MASK_FLC_VALID		BIT(6)
#define BU27010_MASK_WAIT_EN		BIT(3)
#define BU27010_MASK_FIFO_EN		BIT(2)
#define BU27010_MASK_RGB_EN		BIT(1)
#define BU27010_MASK_FLC_EN		BIT(0)

#define BU27010_REG_DATA0_LO		0x50
#define BU27010_REG_DATA1_LO		0x52
#define BU27010_REG_DATA2_LO		0x54
#define BU27010_REG_DATA3_LO		0x56
#define BU27010_REG_DATA_FLICKER_LO	0x56
#define BU27010_MASK_DATA_FLICKER_HI	GENMASK(2, 0)
#define BU27010_REG_FLICKER_COUNT	0x5a
#define BU27010_REG_FIFO_LEVEL_LO	0x5b
#define BU27010_MASK_FIFO_LEVEL_HI	BIT(0)
#define BU27010_REG_FIFO_DATA_LO	0x5d
#define BU27010_REG_FIFO_DATA_HI	0x5e
#define BU27010_MASK_FIFO_DATA_HI	GENMASK(2, 0)
#define BU27010_REG_MANUFACTURER_ID	0x92
#define BU27010_REG_MAX BU27010_REG_MANUFACTURER_ID

struct bu27010_data {
	struct regmap *regmap;
	struct device *dev;
/*	struct iio_trigger *trig;
	struct iio_mount_matrix orientation;
	int64_t timestamp, old_timestamp;

	int irq;
	int inc_reg;
	int ien_reg;

	unsigned int g_range;
	unsigned int state;
	unsigned int odr_ns;

	bool trigger_enabled;
	 *
	 * Prevent toggling the sensor stby/active state (PC1 bit) in the
	 * middle of a configuration, or when the fifo is enabled. Also,
	 * protect the data stored/retrieved from this structure from
	 * concurrent accesses.
	 *
	struct mutex mutex;
	u8 watermark;

	* 3 x 16bit accel data + timestamp *
*	__le16 buffer[8] __aligned(IIO_DMA_MINALIGN);
	struct {
		__le16 channels[3];
		s64 ts __aligned(8);
	} scan;
*/
};


/* Regmap configs */
static const struct regmap_range bu27010_volatile_ranges[] = {
	{
		.range_min = BU27010_REG_MODE_CONTROL5,
		.range_max = BU27010_REG_MODE_CONTROL5,
	}, {
		.range_min = BU27010_REG_DATA0_LO,
		.range_max = BU27010_REG_FIFO_DATA_HI,
	},
};

static const struct regmap_access_table bu27010_volatile_regs = {
	.yes_ranges = &bu27010_volatile_ranges[0],
	.n_yes_ranges = ARRAY_SIZE(bu27010_volatile_ranges),
};

static const struct regmap_range bu27010_read_only_ranges[] = {
	{
		.range_min = BU27010_REG_DATA0_LO,
		.range_max = BU27010_REG_FIFO_DATA_HI,
	}, {
		.range_min = BU27010_REG_MANUFACTURER_ID,
		.range_max = BU27010_REG_MANUFACTURER_ID,
	}
};

static const struct regmap_access_table bu27010_ro_regs = {
	.no_ranges = &bu27010_read_only_ranges[0],
	.n_no_ranges = ARRAY_SIZE(bu27010_read_only_ranges),
};

static const struct regmap_config bu27010_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,

	.max_register	= BU27010_REG_MAX,
	.cache_type	= REGCACHE_RBTREE,
	.volatile_table = &bu27010_volatile_regs,
	.wr_table	= &bu27010_ro_regs,
};

static int bu27010_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct fwnode_handle *fwnode;
	struct bu27010_data *data;
	struct regmap *regmap;
	struct iio_dev *idev;

	if (!i2c->irq) {
		dev_err(dev, "No IRQ configured\n");
		return -EINVAL;
	}

	regmap = devm_regmap_init_i2c(i2c, &bu27010_regmap);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "Failed to initialize Regmap\n");

	fwnode = dev_fwnode(dev);
	if (!fwnode)
		return -ENODEV;

	idev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!idev)
		return -ENOMEM;

	data = iio_priv(idev);

	return 0;
}

static const struct of_device_id bu27010_of_match[] = {
	{ .compatible = "rohm,bu27010", },
	{ }
};
MODULE_DEVICE_TABLE(of, bu27010_of_match);

static struct i2c_driver bu27010_i2c_driver = {
	.driver = {
		.name  = "bu27010-i2c",
		.of_match_table = bu27010_of_match,
	  },
	.probe_new    = bu27010_probe,
};
module_i2c_driver(bu27010_i2c_driver);

MODULE_DESCRIPTION("ROHM BU27010 colour sensor driver");
MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_LICENSE("GPL");
