// SPDX-License-Identifier: GPL-2.0-only
/*
 * BU27008 ROHM Colour Sensor
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
#include <linux/units.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#define BU27008_REG_SYSTEM_CONTROL	0x40
#define BU27008_MASK_SW_RESET		BIT(7)
#define BU27008_MASK_PART_ID		GENMASK(5, 0)
#define BU27008_ID			0x1a
#define BU27008_REG_MODE_CONTROL1	0x41
#define BU27008_MASK_MEAS_MODE		GENMASK(2, 0)
#define BU27008_MEAS_MODE_100MS		0x00
#define BU27008_MEAS_MODE_55MS		0x01
#define BU27008_MEAS_MODE_200MS		0x02
#define BU27008_MEAS_MODE_400MS		0x04

#define BU27008_REG_MODE_CONTROL2	0x42
#define BU27008_MASK_RGBC_GAIN		GENMASK(7, 3)
#define BU27008_SHIFT_RGBC_GAIN		3
#define BU27008_MASK_IR_GAIN_HI		GENMASK(7, 6)
#define BU27008_MASK_IR_GAIN_LO		GENMASK(2, 0)
#define BU27008_SHIFT_IR_GAIN		3
#define BU27008_MASK_IR_GAIN		(BU27008_MASK_IR_GAIN_HI | \
					 BU27008_MASK_IR_GAIN_LO)

#define BU27008_REG_MODE_CONTROL3	0x43
#define BU27008_MASK_VALID		BIT(7)
#define BU27008_MASK_RGB_SEL		GENMASK(3, 2)
#define BU27008_CHAN_RGBC		0
#define BU27008_MASK_INT_EN		BIT(1)
#define BU27008_MASK_MEAS_EN		BIT(0)

#define BU27008_REG_MODE_CONTROL4	0x44
#define BU27008_REG_DATA0_LO		0x50
#define BU27008_REG_DATA1_LO		0x52
#define BU27008_REG_DATA2_LO		0x54
#define BU27008_REG_DATA3_LO		0x56
#define BU27008_REG_DATA3_HI		0x57
#define BU27008_REG_MANUFACTURER_ID	0x92
#define BU27008_REG_MAX BU27008_REG_MANUFACTURER_ID

enum bu27008_channel {
	BU27008_DATA0, /* Always RED */
	BU27008_DATA1, /* Always Green */
	BU27008_DATA2, /* configurable (blue for now) */
	BU27008_DATA3, /* configurable (clear for now) */
	BU27008_NUM_DATA
};

#define BU27008_CHAN_DATA_SIZE		2 /* Each channel has 16bits of data */
#define BU27008_DATA_SIZE (BU27008_NUM_DATA * BU27008_CHAN_DATA_SIZE)

static const unsigned long bu27008_scan_masks[] = {
	GENMASK(BU27008_DATA3, BU27008_DATA0), 0
};

/*
 * All the RGBC channels share the same gain.
 * IR gain can be fine-tuned from the gain set for the RGBC by 2 bits
 * but this would yield quite complex gain setting. Especially since not all
 * bit compinations are supported. And in any case setting GAIN for RGBC will
 * always also change the IR-gain.
 *
 * This is something that should be considered by one who implements the
 * support for the IR channel.
 */
#define BU27008_CHAN(color, data)					\
{									\
	.type = IIO_INTENSITY,						\
	.modified = 1,							\
	.channel2 = IIO_MOD_LIGHT_##color,				\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_all = BIT(IIO_CHAN_INFO_SCALE),		\
	.address = BU27008_REG_##data##_LO,				\
	.scan_index = BU27008_##data,					\
	.scan_type = {							\
		.sign = 's',						\
		.realbits = 16,						\
		.storagebits = 16,					\
		.endianness = IIO_LE,					\
	},								\
}

static const struct iio_chan_spec bu27008_channels[] = {
	BU27008_CHAN(RED, DATA0),
	BU27008_CHAN(GREEN, DATA1),
	BU27008_CHAN(BLUE, DATA2),
	BU27008_CHAN(CLEAR, DATA3),
	IIO_CHAN_SOFT_TIMESTAMP(BU27008_NUM_DATA),
};

struct bu27008_data {
	struct regmap *regmap;
	struct iio_trigger *trig;
	struct device *dev;
	int64_t timestamp, old_timestamp;

	int irq;

	/*
	 * Prevent toggling the trigger enable/disable when
	 * Prevent toggling the sensor stby/active state (PC1 bit) in the
	 * middle of a configuration, or when the fifo is enabled. Also,
	 * protect the data stored/retrieved from this structure from
	 * concurrent accesses.
	 */
	struct mutex mutex;

	struct {
		__le16 channels[4];
		s64 ts __aligned(8);
	} scan;

};

static int bu27008_chan_read_data(struct bu27008_data *data, int reg, int *val)
{
	int ret;
	__le16 tmp;

	ret = regmap_bulk_read(data->regmap, reg, &tmp, sizeof(tmp));
	if (ret)
		dev_err(data->dev, "Reading channel data failed\n");

	*val = le16_to_cpu(tmp);

	return ret;
}

struct bu27008_gain {
	int reg;
	int gain;
};

#define BU27008_GAIN(_reg, _gain)	\
{					\
	.reg = (_reg),			\
	.gain = (_gain),		\
}

static const struct bu27008_gain bu27008_gain_tbl[] = {
	BU27008_GAIN(0x00, 1),		/* 1x */
	BU27008_GAIN(0x08, 4),		/* 4x */
	BU27008_GAIN(0x09, 8),		/* 8x */
	BU27008_GAIN(0x0a, 16),		/* 16x */
	BU27008_GAIN(0x0b, 32),		/* 32x */
	BU27008_GAIN(0x0c, 64),		/* 64x */
	BU27008_GAIN(0x18, 256),	/* 256x */
	BU27008_GAIN(0x19, 512),	/* 512x */
	BU27008_GAIN(0x1a, 1024),	/* 1024x */
};

static int gain2scale_hi(int gain)
{
	return 4 / gain;
}

static int gain2scale_lo(int gain)
{
	if (gain < 8)
		return 0;

	return GIGA / (gain >> 2);
}

static bool bu27008_valid_gain(struct bu27008_data *data, int gain)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bu27008_gain_tbl); i++)
		if (bu27008_gain_tbl[i].gain == gain)
			return true;

	return false;
}



struct bu27008_gain_scale_linear {
	u64 linscale;
	int gain;
};

#define GAINSCALE_HI(_g, _s)		\
{					\
	.gain = (_g),			\
	.linscale = (_s * GIGA)		\
}

#define GAINSCALE_LO(_g, _sdiv)		\
{					\
	.gain = (_g),			\
	.linscale = GIGA/(_sdiv)	\
}

static const struct bu27008_gain_scale_linear bu27008_virtual_gain_scale_table[] = {
	GAINSCALE_HI(1, 4),
	GAINSCALE_HI(4, 1),
	GAINSCALE_LO(8, 2),
	GAINSCALE_HI(16, 4),
	GAINSCALE_HI(32, 8),
	GAINSCALE_LO(64, 16),
	GAINSCALE_LO(256, 64),
	GAINSCALE_LO(512, 128),
	GAINSCALE_LO(1024, 256),
};

static int find_gain_by_linearized_scale(u64 linscale)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bu27008_virtual_gain_scale_table); i++)
		if (bu27008_virtual_gain_scale_table[i].linscale == linscale)
			return bu27008_virtual_gain_scale_table[i].gain;

	return -EINVAL;
}

static int bu27008_get_gain(struct bu27008_data *data, int *gain)
{
	int ret, rval, i;

	ret = regmap_read(data->regmap, BU27008_REG_MODE_CONTROL2, &rval);
	if (ret)
		return ret;

	rval &= BU27008_MASK_RGBC_GAIN;
	rval >>= BU27008_SHIFT_RGBC_GAIN;

	for (i = 0; i < ARRAY_SIZE(bu27008_gain_tbl); i++)
		if (bu27008_gain_tbl[i].reg == rval) {
			*gain = bu27008_gain_tbl[i].gain;

			return 0;
		}

	return -EINVAL;
}

static int bu27008_set_gain(struct bu27008_data *data, int gain)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(bu27008_gain_tbl); i++)
		if (bu27008_gain_tbl[i].gain == gain) {
			int rval;

			rval = bu27008_gain_tbl[i].reg;
			rval <<= BU27008_SHIFT_RGBC_GAIN;

			rval |= bu27008_gain_tbl[i].reg &
				BU27008_MASK_IR_GAIN_LO;

			return regmap_update_bits(data->regmap,
						  BU27008_REG_MODE_CONTROL2,
						  BU27008_MASK_RGBC_GAIN, rval);
		}

	return -EINVAL;
}

struct bu27008_int_time {
	int scale_multiplier;
	int int_time_us;
};

static const struct bu27008_int_time bu27008_int_times[] = {
	{
		.scale_multiplier = 8,
		.int_time_us = 50000,
	}, {
		.scale_multiplier = 4,
		.int_time_us = 100000,
	}, {
		.scale_multiplier = 2,
		.int_time_us = 200000,
	}, {
		.scale_multiplier = 1,
		.int_time_us = 400000,
	},
};

static int bu27008_get_int_time(struct bu27008_data *data)
{
	int ret, tmp;

	ret = regmap_read(data->regmap, BU27008_REG_MODE_CONTROL1, &tmp);
	if (ret)
		return ret;

	switch (tmp & BU27008_MASK_MEAS_MODE) {
	case BU27008_MEAS_MODE_100MS:
		return 100000;
	/* Data sheet says 55 mS, vendor code uses 50 mS for all computations */
	case BU27008_MEAS_MODE_55MS:
		return 50000;
	case BU27008_MEAS_MODE_200MS:
		return 200000;
	case BU27008_MEAS_MODE_400MS:
		return 400000;
	}

	dev_err(data->dev, "unknown meas mode 0x%lx\n",
		tmp & BU27008_MASK_MEAS_MODE);

	return -EINVAL;
}

static int bu27008_get_int_time_multiplier(struct bu27008_data *data)
{
	int ret;

	ret = bu27008_get_int_time(data);
	if (ret < 0)
		return ret;

	return 400000 / ret;
}

static int bu27008_get_scale(struct bu27008_data *data, int *val, int *val2)
{
	int gain, ret;

	ret = bu27008_get_gain(data, &gain);
	if (ret)
		return ret;

	ret = bu27008_get_int_time_multiplier(data);
	if (ret < 0)
		return ret;

	gain *= ret;

	*val = gain2scale_hi(gain);
	*val2 = gain2scale_lo(gain);

	return 0;
}

static int bu27008_set_int_time(struct bu27008_data *data, int time)
{
	int reg;

	switch (time) {
	case 50000:
		reg = BU27008_MEAS_MODE_55MS;
		break;
	case 100000:
		reg = BU27008_MEAS_MODE_100MS;
		break;
	case 200000:
		reg = BU27008_MEAS_MODE_200MS;
		break;
	case 400000:
		reg = BU27008_MEAS_MODE_400MS;
		break;
	default:
		dev_err(data->dev, "unsupported integration time %u\n",
			time);
		return -EINVAL;
	}

	return regmap_update_bits(data->regmap, BU27008_REG_MODE_CONTROL1,
				  BU27008_MASK_MEAS_MODE, reg);
}

static int bu27008_validate_int_time(int time_us)
{
	int i;

	if (time_us == 55000)
		return 50000;

	for (i = 1; i < ARRAY_SIZE(bu27008_int_times); i++)
		if (bu27008_int_times[i].int_time_us == time_us)
			break;

	if (i == ARRAY_SIZE(bu27008_int_times))
		return -EINVAL;

	return bu27008_int_times[i].int_time_us;
}

/* Try to change the time so that the scale is maintained */
static int bu27008_try_set_int_time(struct bu27008_data *data, int time_us)
{
	int ret, int_time_old, int_time_new, old_gain, new_gain;

	int_time_old = bu27008_get_int_time(data);
	if (int_time_old < 0)
		return int_time_old;

	if (time_us == int_time_old)
		return 0;

	int_time_new = bu27008_validate_int_time(time_us);

	if (int_time_new < 0) {
		dev_err(data->dev, "Unsupported integration time %u\n",
			time_us);

		return int_time_new;
	}
	ret = bu27008_get_gain(data, &old_gain);
	if (ret)
		return ret;

	new_gain = old_gain * int_time_old / int_time_new;

	if (!bu27008_valid_gain(data, new_gain)) {
		int scale1, scale2;

		bu27008_get_scale(data, &scale1, &scale2);
		dev_err(data->dev,
			"can't support time %u with scale %u %u\n", time_us,
			scale1, scale2);

		return -EINVAL;
	}

	/*
	 * Yay, if we got here the new integration time can be supported while
	 * keeping the scale of channels intact by tuning the gains.
	 * NOTE: This operation is not atomic.
	 */

	ret = bu27008_set_gain(data, new_gain);
	if (ret)
		return ret;

	return bu27008_set_int_time(data, int_time_new);
}

static int bu27008_read_raw(struct iio_dev *idev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct bu27008_data *data = iio_priv(idev);
	int busy, ret = -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		busy = iio_device_claim_direct_mode(idev);
		if (busy)
			return -EBUSY;

		pr_info("reading address chan->address 0x%lx\n", chan->address);
		ret = bu27008_chan_read_data(data, chan->address, val);
		if (!ret)
			ret = IIO_VAL_INT;

		iio_device_release_direct_mode(idev);

		return ret;

	case IIO_CHAN_INFO_SCALE:
		ret = bu27008_get_scale(data, val, val2);
		if (ret)
			return ret;

		ret = IIO_VAL_INT_PLUS_NANO;
	}

	return ret;
}

#define for_each_bu27008_time(_tim, _mul) \
	for ((_tim) = 400000, (_mul) = 1; _tim >= 50000; _tim >>= 1, _mul <<= 1)

static int bu27008_set_scale(struct bu27008_data *data, int chan,
			    int val, int val2)
{
	int mul, gain, ret;
	u64 scale_linear;
/*
	if (chan != BU27008_CHAN_DATA0 &&
	    chan != BU27008_CHAN_DATA1 &&
	    chan != BU27008_CHAN_DATA2)
		return -EINVAL;
*/
	mul = bu27008_get_int_time_multiplier(data);
	if (mul < 0)
		return mul;

	scale_linear = val * GIGA + val2;

	do_div(scale_linear, mul);

	gain = find_gain_by_linearized_scale(scale_linear);
	if (gain < 0) {
		int tim, mul_new;

		for_each_bu27008_time(tim, mul_new) {
			scale_linear = val * GIGA + val2;
			do_div(scale_linear, mul_new);

			gain = find_gain_by_linearized_scale(scale_linear);
			if (gain > 0) {
				mutex_lock(&data->mutex);
				ret = bu27008_set_int_time(data, tim);
				if (!ret)
					ret = bu27008_set_gain(data, gain);
				mutex_unlock(&data->mutex);

				return ret;
			}
		}
	} else {
		return bu27008_set_gain(data, gain);
	}

	dev_err(data->dev, "Can't set scale %u %u\n", val, val2);
	return -EINVAL;
}




static int bu27008_write_raw(struct iio_dev *idev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long mask)
{
	struct bu27008_data *data = iio_priv(idev);
	int ret;

	/*
	 * We should not allow changing scale when measurement is ongoing.
	 * This could make values in buffer inconsistent.
	 */
	ret = iio_device_claim_direct_mode(idev);
	if (ret)
		return ret;

	switch (mask) {

	case IIO_CHAN_INFO_SCALE:
		ret = bu27008_set_scale(data, chan->channel, val, val2);
		break;

	case IIO_CHAN_INFO_INT_TIME:
		ret =  bu27008_try_set_int_time(data, val2);
		break;

	default:
		ret = -EINVAL;
	}
	iio_device_release_direct_mode(idev);

	return ret;
}

static int bu27008_validate_trigger(struct iio_dev *idev,
				   struct iio_trigger *trig)
{
	struct bu27008_data *data = iio_priv(idev);

	if (data->trig != trig)
		return -EINVAL;

	return 0;
}

static const struct iio_info bu27008_info = {
	.read_raw = &bu27008_read_raw,
	.write_raw = &bu27008_write_raw,
//	.read_avail = &kx022a_read_avail,

	.validate_trigger	= bu27008_validate_trigger,
};

/* Regmap configs */
static const struct regmap_range bu27008_volatile_ranges[] = {
	{
		.range_min = BU27008_REG_MODE_CONTROL4,
		.range_max = BU27008_REG_MODE_CONTROL4,
	}, {
		.range_min = BU27008_REG_DATA0_LO,
		.range_max = BU27008_REG_DATA3_HI,
	},
};

static const struct regmap_access_table bu27008_volatile_regs = {
	.yes_ranges = &bu27008_volatile_ranges[0],
	.n_yes_ranges = ARRAY_SIZE(bu27008_volatile_ranges),
};

static const struct regmap_range bu27008_read_only_ranges[] = {
	{
		.range_min = BU27008_REG_DATA0_LO,
		.range_max = BU27008_REG_DATA3_HI,
	}, {
		.range_min = BU27008_REG_MANUFACTURER_ID,
		.range_max = BU27008_REG_MANUFACTURER_ID,
	}
};

static const struct regmap_access_table bu27008_ro_regs = {
	.no_ranges = &bu27008_read_only_ranges[0],
	.n_no_ranges = ARRAY_SIZE(bu27008_read_only_ranges),
};

static const struct regmap_config bu27008_regmap = {
	.reg_bits	= 8,
	.val_bits	= 8,

	.max_register	= BU27008_REG_MAX,
	.cache_type	= REGCACHE_RBTREE,
	.volatile_table = &bu27008_volatile_regs,
	.wr_table	= &bu27008_ro_regs,
};

static int bu27008_chip_init(struct bu27008_data *data)
{
	int ret;

	/* Reset */
	ret = regmap_update_bits(data->regmap, BU27008_REG_SYSTEM_CONTROL,
			   BU27008_MASK_SW_RESET, BU27008_MASK_SW_RESET);
	if (ret)
		return dev_err_probe(data->dev, ret, "Sensor reset failed\n");

	/*
	 * Delay to allow IC to initialize. We don't care if we delay
	 * for more than 1 ms so msleep() is Ok. We just don't want to
	 * block
	 */
	msleep(1);

	ret = regmap_update_bits(data->regmap, BU27008_REG_MODE_CONTROL3,
				 BU27008_MASK_RGB_SEL, BU27008_CHAN_RGBC);
	if (ret)
		dev_err(data->dev, "Channel setup failed\n");

	return ret;
}

static int bu27008_set_drdy_irq(struct bu27008_data *data, bool state)
{
	if (state)
		return regmap_set_bits(data->regmap, BU27008_REG_MODE_CONTROL3,
					BU27008_MASK_INT_EN);
	return regmap_clear_bits(data->regmap, BU27008_REG_MODE_CONTROL3,
				 BU27008_MASK_INT_EN);
}

static int bu27008_trigger_set_state(struct iio_trigger *trig,
				     bool state)
{
	struct bu27008_data *data = iio_trigger_get_drvdata(trig);
	int ret = 0;

//	mutex_lock(&data->mutex);

	if (data->trigger_enabled == state)
		return 0;
	//	goto unlock_out;
/*
	if (data->state & KX022A_STATE_FIFO) {
		dev_warn(data->dev, "Can't set trigger when FIFO enabled\n");
		ret = -EBUSY;
		goto unlock_out;
	}

	ret = kx022a_turn_on_off_unlocked(data, false);
	if (ret)
		goto unlock_out;
*/
	data->trigger_enabled = state;
	ret = bu27008_set_drdy_irq(data, state);
	if (ret)
		dev_err(data->dev, "Failed to set trigger state\n");
		//goto unlock_out;

//	ret = kx022a_turn_on_off_unlocked(data, true);
//
//unlock_out:
//	mutex_unlock(&data->mutex);

	return ret;
}

static const struct iio_trigger_ops bu27008_trigger_ops = {
	.set_trigger_state = bu27008_trigger_set_state,
};

static irqreturn_t bu27008_irq_handler(int irq, void *private)
{
	struct iio_dev *idev = private;
	struct bu27008_data *data = iio_priv(idev);

	data->old_timestamp = data->timestamp;
	data->timestamp = iio_get_time_ns(idev);

	if (/* data->state & KX022A_STATE_FIFO ||*/ data->trigger_enabled)
		return IRQ_WAKE_THREAD;

	return IRQ_NONE;
}

static irqreturn_t bu27008_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *idev = pf->indio_dev;
	struct bu27008_data *data = iio_priv(idev);
	int ret;

	ret = regmap_bulk_read(data->regmap, BU27008_REG_DATA0_LO, data->buffer,
			       BU27008_DATA_SIZE);
	if (ret < 0)
		goto err_read;

	iio_push_to_buffers_with_timestamp(idev, data->buffer, pf->timestamp);
err_read:
	iio_trigger_notify_done(idev->trig);

	return IRQ_HANDLED;
}

static irqreturn_t bu27008_irq_thread_handler(int irq, void *private)
{
	struct iio_dev *idev = private;
	struct bu27008_data *data = iio_priv(idev);
	irqreturn_t ret = IRQ_NONE;

	mutex_lock(&data->mutex);

	if (data->trigger_enabled) {
		iio_trigger_poll_chained(data->trig);
		ret = IRQ_HANDLED;
	}

	mutex_unlock(&data->mutex);

	return ret;
}

static int bu27008_meas_set(struct bu27008_data *data, bool enable)
{
	if (enable)
		return regmap_set_bits(data->regmap, BU27008_REG_MODE_CONTROL3,
				       BU27008_MASK_MEAS_EN);

	return regmap_clear_bits(data->regmap, BU27008_REG_MODE_CONTROL3,
				 BU27008_MASK_MEAS_EN);
}

static int bu27008_meas_en(struct bu27008_data *data)
{
	return bu27008_meas_set(data, true);
}

static int bu27008_meas_dis(struct bu27008_data *data)
{
	return bu27008_meas_set(data, false);
}

static int bu27008_buffer_preenable(struct iio_dev *idev)
{
	struct bu27008_data *data = iio_priv(idev);

	return bu27008_meas_en(data);
}

static int bu27008_buffer_postdisable(struct iio_dev *idev)
{
	struct bu27008_data *data = iio_priv(idev);

	return bu27008_meas_dis(data);
}

static const struct iio_buffer_setup_ops bu27008_buffer_ops = {
	.preenable = bu27008_buffer_preenable,
	.postdisable = bu27008_buffer_postdisable,
};

static int bu27008_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct iio_trigger *indio_trig;
	struct fwnode_handle *fwnode;
	struct bu27008_data *data;
	struct regmap *regmap;
	unsigned int part_id;
	struct iio_dev *idev;
	char *name;
	int ret;

	if (!i2c->irq) {
		dev_err(dev, "No IRQ configured\n");
		return -EINVAL;
	}

	regmap = devm_regmap_init_i2c(i2c, &bu27008_regmap);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "Failed to initialize Regmap\n");

	fwnode = dev_fwnode(dev);
	if (!fwnode)
		return -ENODEV;

	idev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!idev)
		return -ENOMEM;

	ret = devm_regulator_get_enable_optional(dev, "vdd");
	if (ret != -ENODEV)
		return dev_err_probe(dev, ret, "Failed to get regulator\n");

	data = iio_priv(idev);

	ret = regmap_read(regmap, BU27008_REG_SYSTEM_CONTROL, &part_id);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to access sensor\n");

	part_id &= BU27008_MASK_PART_ID;

	if (part_id != BU27008_ID) {
		dev_err(dev, "unsupported device 0x%x\n", part_id);
		return -EINVAL;
	}

	data->regmap = regmap;
	data->dev = dev;
	data->irq = i2c->irq;
//	data->odr_ns = KX022A_DEFAULT_PERIOD_NS;
	mutex_init(&data->mutex);

	idev->channels = bu27008_channels;
	idev->num_channels = ARRAY_SIZE(bu27008_channels);
	idev->name = "bu27008-light";
	idev->info = &bu27008_info;
	idev->modes = INDIO_DIRECT_MODE | INDIO_BUFFER_SOFTWARE;
	idev->available_scan_masks = bu27008_scan_masks;

	ret = bu27008_chip_init(data);
	if (ret)
		return ret;
	ret = devm_iio_triggered_buffer_setup_ext(dev, idev,
						  &iio_pollfunc_store_time,
						  bu27008_trigger_handler,
						  IIO_BUFFER_DIRECTION_IN,
						  &bu27008_buffer_ops,
//						  kx022a_fifo_attributes);
						  NULL);

	if (ret)
		return dev_err_probe(data->dev, ret,
				     "iio_triggered_buffer_setup_ext FAIL\n");

	indio_trig = devm_iio_trigger_alloc(dev, "%sdata-rdy-dev%d", idev->name,
					    iio_device_id(idev));
	if (!indio_trig)
		return -ENOMEM;

	data->trig = indio_trig;

	indio_trig->ops = &bu27008_trigger_ops;
	iio_trigger_set_drvdata(indio_trig, data);

	/*
	 * No need to check for NULL. request_threaded_irq() defaults to
	 * dev_name() should the alloc fail.
	 */
	name = devm_kasprintf(data->dev, GFP_KERNEL, "%s-bu27008",
			      dev_name(data->dev));

	ret = devm_request_threaded_irq(data->dev, i2c->irq, bu27008_irq_handler,
					&bu27008_irq_thread_handler,
					IRQF_ONESHOT, name, idev);
	if (ret)
		return dev_err_probe(data->dev, ret, "Could not request IRQ\n");


	ret = devm_iio_trigger_register(dev, indio_trig);
	if (ret)
		return dev_err_probe(data->dev, ret,
				     "Trigger registration failed\n");

	ret = devm_iio_device_register(data->dev, idev);
	if (ret < 0)
		return dev_err_probe(dev, ret,
				     "Unable to register iio device\n");

	return ret;
}

static const struct of_device_id bu27008_of_match[] = {
	{ .compatible = "rohm,bu27008", },
	{ }
};
MODULE_DEVICE_TABLE(of, bu27008_of_match);

static struct i2c_driver bu27008_i2c_driver = {
	.driver = {
		.name  = "bu27008-i2c",
		.of_match_table = bu27008_of_match,
	  },
	.probe_new    = bu27008_probe,
};
module_i2c_driver(bu27008_i2c_driver);

MODULE_DESCRIPTION("ROHM BU27008 colour sensor driver");
MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_LICENSE("GPL");
