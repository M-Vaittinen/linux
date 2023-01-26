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

enum {
	BU27010_DATA0, /* Always RED */
	BU27010_DATA1, /* Always Green */
	BU27010_DATA2, /* configurable (blue for now) */
	BU27010_DATA3, /* configurable (clear for now) */
	BU27010_FLICKER, /* flickering fifo */
	BU27010_NUM_CHANS
};

#define BU27008_CHAN_DATA_SIZE		2 /* Each channel has 16bits of data */
#define BU27008_DATA_SIZE (BU27008_NUM_CHANS * BU27008_CHAN_DATA_SIZE)

static const unsigned long bu27008_scan_masks[] = {
	GENMASK(BU27008_DATA3, BU27008_DATA0), 0
};

/*
 * Available scales with gain 1x - 1024x, timings 55, 100, 200, 400 mS
 * Time impacts to gain: 1x, 2x, 4x, 8x.
 *
 * => Max total gain is HWGAIN * gain by integration time (8 * 1024) = 8192
 *
 * Using NANO precision for scale we must use scale 16x corresponding gain 1x
 * to avoid precision loss.
 */
#define BU270010_SCALE_1X 64

/* See the data sheet for the "Gain Setting" table */
#define BU27010_GSEL_1X		0x00	/* 000000 */
#define BU27010_GSEL_4X		0x08	/* 001000 */
#define BU27010_GSEL_16X	0x09	/* 001001 */
#define BU27010_GSEL_64X	0x0e	/* 001110 */
#define BU27010_GSEL_256X	0x1e	/* 011110 */
#define BU27010_GSEL_1024X	0x2e	/* 101110 */
#define BU27010_GSEL_4096X	0x3f	/* 111111 */

static const struct iio_gain_sel_pair bu27008_gains[] = {
	GAIN_SCALE_GAIN(1, BU27010_GSEL_1X),
	GAIN_SCALE_GAIN(4, BU27010_GSEL_4X),
	GAIN_SCALE_GAIN(16, BU27010_GSEL_16X),
	GAIN_SCALE_GAIN(64, BU27010_GSEL_64X),
	GAIN_SCALE_GAIN(256, BU27010_GSEL_256X),
	GAIN_SCALE_GAIN(1024, BU27010_GSEL_1024X),
	GAIN_SCALE_GAIN(4096, BU27010_GSEL_4096X),
};

#define BU27010_MEAS_MODE_100MS		0x00
#define BU27010_MEAS_MODE_55MS		0x01
#define BU27010_MEAS_MODE_200MS		0x02
#define BU27010_MEAS_MODE_400MS		0x04

static const struct iio_itime_sel_mul bu27010_itimes[] = {
        GAIN_SCALE_ITIME_US(400000, BU27010_MEAS_MODE_400MS, 8),
        GAIN_SCALE_ITIME_US(200000, BU27010_MEAS_MODE_200MS, 4),
        GAIN_SCALE_ITIME_US(100000, BU27010_MEAS_MODE_100MS, 2),
        GAIN_SCALE_ITIME_US(50000, BU27010_MEAS_MODE_55MS, 1),
};

/*
 * GAIN setting for the bu27010 stays in same odd "Hey, lets share some gain
 * setting bits for all or some channels" road as did bu27034 and bu27008.
 *
 * Gain is selected using 6 bits - and only some bit combinations are valid.
 * (see the defines BU27010_GSEL_<gain>X above).
 * What makes this odd is that four high bits of the gain setting are common
 * for all channels - but the low 2 bits can be set independently for each
 * channel. I am unsure of this makes  sense because - if we allow setting
 * common bits, we will change GAIN for all channels. If we 'fix' the high bits
 * and allow only setting two lowest bits (which were independetly changeable
 * for each channel) we can only support two different valid GAIN bit
 * combinations:
 * 001000 and 001001 - which represent gains 4X and 16X.
 * ruling out all other gains.
 *
 * So, we ignore the fact that in theory we could support some per channel gain
 * configurations and just say the GAIN is shared by all channels and always set
 * the same gain for all channels.
 *
 * FLICKER detection has own gain setting but currently we don't support the
 * flicker detection at all.
 *
 * This is something that should be considered by one who implements the
 * support for the IR channel.
 */
#define BU27010_CHAN(color, data)					\
{									\
	.type = IIO_INTENSITY,						\
	.modified = 1,							\
	.channel2 = IIO_MOD_LIGHT_##color,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SCALE) |		\
				   BIT(IIO_CHAN_INFO_INT_TIME),		\
	.info_mask_shared_by_all_available = /* BIT(IIO_CHAN_INFO_SCALE) TODO: How to make IR-scale*/	\
					BIT(IIO_CHAN_INFO_INT_TIME),	\
	.address = BU27010_REG_##data##_LO,				\
	.scan_index = BU27010_##data,					\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.endianness = IIO_LE,					\
	},								\
}

/* TODO: Fix this to same as bu27008 */
static const struct iio_chan_spec bu27010_channels[] = {
	BU27010_CHAN(RED, DATA0),
	BU27010_CHAN(GREEN, DATA1),
	BU27010_CHAN(BLUE, DATA2),
	BU27010_CHAN(CLEAR, DATA3),
	IIO_CHAN_SOFT_TIMESTAMP(BU27008_NUM_CHANS),
};

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
