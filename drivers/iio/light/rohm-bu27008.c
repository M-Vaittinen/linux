// SPDX-License-Identifier: GPL-2.0-only
/*
 * BU27008 ROHM Colour Sensor
 *
 * Copyright (c) 2023, ROHM Semiconductor.
 */

#include <linux/bits.h>
#include <linux/bitfield.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/units.h>

#include <linux/iio/iio.h>
#include <linux/iio/iio-gts-helper.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/triggered_buffer.h>

#define BU27008_REG_SYSTEM_CONTROL	0x40
#define BU27008_MASK_SW_RESET		BIT(7)
#define BU27008_MASK_PART_ID		GENMASK(5, 0)
#define BU27008_ID			0x1a
#define BU27008_REG_MODE_CONTROL1	0x41
#define BU27008_MASK_MEAS_MODE		GENMASK(2, 0)

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

/*
 * Max amplification is (HWGAIN * MAX integration-time multiplier) 1024 * 8
 * = 8192. With NANO scale we get rid of accuracy loss when we start with the
 * scale 16.0 for HWGAIN1, INT-TIME 55 mS. This way the nano scale for MAX
 * total gain 8192 will be 1953125
 */

enum {
	BU27008_DATA0, /* Always RED */
	BU27008_DATA1, /* Always Green */
	BU27008_DATA2, /* configurable (blue for now) */
	BU27008_DATA3, /* configurable (clear for now) */
	BU27008_NUM_CHANS
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
#define BU27008_SCALE_1X 16

/* See the data sheet for the "Gain Setting" table */
#define BU27008_GSEL_1X		0x00
#define BU27008_GSEL_4X		0x08
#define BU27008_GSEL_8X		0x09
#define BU27008_GSEL_16X	0x0a
#define BU27008_GSEL_32X	0x0b
#define BU27008_GSEL_64X	0x0c
#define BU27008_GSEL_256X	0x18
#define BU27008_GSEL_512X	0x19
#define BU27008_GSEL_1024X	0x1a

static const struct iio_gain_sel_pair bu27008_gains[] = {
	GAIN_SCALE_GAIN(1, BU27008_GSEL_1X),
	GAIN_SCALE_GAIN(4, BU27008_GSEL_4X),
	GAIN_SCALE_GAIN(8, BU27008_GSEL_8X),
	GAIN_SCALE_GAIN(16, BU27008_GSEL_16X),
	GAIN_SCALE_GAIN(32, BU27008_GSEL_32X),
	GAIN_SCALE_GAIN(64, BU27008_GSEL_64X),
	GAIN_SCALE_GAIN(256, BU27008_GSEL_256X),
	GAIN_SCALE_GAIN(512, BU27008_GSEL_512X),
	GAIN_SCALE_GAIN(1024, BU27008_GSEL_1024X),
};

#define BU27008_MEAS_MODE_100MS		0x00
#define BU27008_MEAS_MODE_55MS		0x01
#define BU27008_MEAS_MODE_200MS		0x02
#define BU27008_MEAS_MODE_400MS		0x04

static const struct iio_itime_sel_mul bu27008_itimes[] = {
        GAIN_SCALE_ITIME_US(400000, BU27008_MEAS_MODE_400MS, 8),
        GAIN_SCALE_ITIME_US(200000, BU27008_MEAS_MODE_200MS, 4),
        GAIN_SCALE_ITIME_US(100000, BU27008_MEAS_MODE_100MS, 2),
        GAIN_SCALE_ITIME_US(50000, BU27008_MEAS_MODE_55MS, 1),
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
	IIO_CHAN_SOFT_TIMESTAMP(BU27008_NUM_CHANS),
};

struct bu27008_data {
	struct regmap *regmap;
	struct iio_trigger *trig;
	struct device *dev;
	struct iio_gts gts;
	int64_t timestamp, old_timestamp;

	int irq;

	/*
	 * Prevent changing gain/time when scale is read/written. Prevent
	 * changing gain/time when raw data is read.
	 */
	struct mutex mutex;
	bool trigger_enabled;

	__le16 buffer[4];

	struct {
		__le16 channels[4];
		s64 ts __aligned(8);
	} scan;

};

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
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = BU27008_REG_MAX,
	.cache_type = REGCACHE_RBTREE,
	.volatile_table = &bu27008_volatile_regs,
	.wr_table = &bu27008_ro_regs,
};



#define BU27008_MAX_VALID_RESULT_WAIT_US	50000
#define BU27008_VALID_RESULT_WAIT_QUANTA_US	1000

static int bu27008_chan_read_data(struct bu27008_data *data, int reg, int *val)
{
	int ret, valid;
	__le16 tmp;

	ret = regmap_read_poll_timeout(data->regmap, BU27008_REG_MODE_CONTROL3,
				       valid, (valid & BU27008_MASK_VALID),
				       BU27008_VALID_RESULT_WAIT_QUANTA_US,
				       BU27008_MAX_VALID_RESULT_WAIT_US);
	if (ret)
		return ret;

	ret = regmap_bulk_read(data->regmap, reg, &tmp, sizeof(tmp));
	if (ret)
		dev_err(data->dev, "Reading channel data failed\n");

	*val = le16_to_cpu(tmp);

	return ret;
}

static int bu27008_get_gain(struct bu27008_data *data, int *gain)
{
	int ret, sel;

	ret = regmap_read(data->regmap, BU27008_REG_MODE_CONTROL2, &sel);
	if (ret)
		return ret;

	sel = FIELD_GET(BU27008_MASK_RGBC_GAIN, sel);
//	sel &= BU27008_MASK_RGBC_GAIN;
//	sel >>= BU27008_SHIFT_RGBC_GAIN;

	ret = iio_gts_find_gain_by_sel(&data->gts, sel);
	if (ret < 0) {
		dev_err(data->dev, "unknown gain value 0x%x\n", sel);

		return ret;
	}

	*gain = ret;

	return 0;
}

static int bu27008_write_gain_sel(struct bu27008_data *data, int sel)
{
	int regval;

	regval = FIELD_PREP(BU27008_MASK_IR_GAIN_LO, sel);

	/*
	 * We do always set also the LOW bits of IR-gain because othervice we
	 * would risk resulting an invalid GAIN register value.
	 */
	regval |= sel & BU27008_MASK_IR_GAIN_LO;

	return regmap_update_bits(data->regmap, BU27008_REG_MODE_CONTROL2,
				  BU27008_MASK_RGBC_GAIN, sel);
}

static int bu27008_set_gain(struct bu27008_data *data, int gain)
{
	int ret;

	ret = iio_gts_find_sel_by_gain(&data->gts, gain);
	if (ret < 0)
		return ret;

	return bu27008_write_gain_sel(data, ret);
}

static int bu27008_get_int_time_sel(struct bu27008_data *data, int *sel)
{
	int ret;

	ret = regmap_read(data->regmap, BU27008_REG_MODE_CONTROL1, sel);
	*sel &= BU27008_MASK_MEAS_MODE;

	return ret;
}

static int bu27008_set_int_time_sel(struct bu27008_data *data, int sel)
{
		return regmap_update_bits(data->regmap,
					  BU27008_REG_MODE_CONTROL1,
					  BU27008_MASK_MEAS_MODE, sel);
}

static int bu27008_get_int_time(struct bu27008_data *data)
{
	int ret, sel;

	ret = bu27008_get_int_time_sel(data, &sel);
	if (ret)
		return ret;

	return iio_gts_find_int_time_by_sel(&data->gts,
					    sel & BU27008_MASK_MEAS_MODE);
}
/*
static int bu27008_get_int_time_multiplier(struct bu27008_data *data)
{
	int ret;

	ret = bu27008_get_int_time(data);
	if (ret < 0)
		return ret;

	return 400000 / ret;
}
*/
static int _bu27008_get_scale(struct bu27008_data *data, int *val, int *val2)
{
	int gain, ret;

	ret = bu27008_get_gain(data, &gain);
	if (ret)
		return ret;

	ret = bu27008_get_int_time(data);
	if (ret < 0)
		return ret;

	return iio_gts_get_scale(&data->gts, gain, ret, val, val2);
}

static int bu27008_get_scale(struct bu27008_data *data, int *val, int *val2)
{
	int ret;

	mutex_lock(&data->mutex);
	ret = _bu27008_get_scale(data, val, val2);
	mutex_unlock(&data->mutex);

	return ret;
}

static int bu27008_set_int_time(struct bu27008_data *data, int time)
{
	int ret;

	ret = iio_gts_find_sel_by_int_time(&data->gts, time);
	if (ret < 0)
		return ret;

	/*
	 * TODO: Do we need data->cached = false; ? See how the raw_read
	 * gets implemented
	 */

	return regmap_update_bits(data->regmap, BU27008_REG_MODE_CONTROL1,
				  BU27008_MASK_MEAS_MODE, ret);
}

static int bu27008_validate_int_time(struct bu27008_data *data, int time_us)
{
	if (time_us == 55000)
		return 50000;

	if (iio_gts_valid_time(&data->gts, time_us))
		return time_us;

	return -EINVAL;
}

/* Try to change the time so that the scale is maintained */
static int bu27008_try_set_int_time(struct bu27008_data *data, int time_us)
{
	int ret, old_time_sel, new_time_sel, int_time_new,  old_gain, new_gain;

	mutex_lock(&data->mutex);

	ret = bu27008_get_int_time_sel(data, &old_time_sel);
	if (ret < 0)
		goto unlock_out;

	int_time_new = bu27008_validate_int_time(data, time_us);

	if (int_time_new < 0) {
		dev_err(data->dev, "Unsupported integration time %u\n",
			time_us);

		ret = int_time_new;
		goto unlock_out;
	}
	new_time_sel = iio_gts_find_sel_by_int_time(&data->gts, int_time_new);
	if (new_time_sel == old_time_sel) {
		ret = 0;
		goto unlock_out;
	}

	ret = bu27008_get_gain(data, &old_gain);
	if (ret)
		goto unlock_out;

	ret = iio_gts_find_new_gain_sel_by_old_gain_time(&data->gts, old_gain,
				old_time_sel, new_time_sel, &new_gain);

	if (ret) {
		int scale1, scale2;

		bu27008_get_scale(data, &scale1, &scale2);
		dev_err(data->dev,
			"Can't support time %u with current scale %u %u\n",
			time_us, scale1, scale2);

		goto unlock_out;
	}

	ret = bu27008_set_gain(data, new_gain);
	if (ret)
		goto unlock_out;

	ret = bu27008_set_int_time(data, int_time_new);

unlock_out:
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

static int bu27008_read_raw(struct iio_dev *idev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long mask)
{
	struct bu27008_data *data = iio_priv(idev);
	int busy, ret = -EINVAL;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
	{
		int int_time;

		busy = iio_device_claim_direct_mode(idev);
		if (busy)
			return -EBUSY;

		mutex_lock(&data->mutex);
		bu27008_meas_en(data);
		int_time = bu27008_get_int_time(data);
		if (int_time < 0)
			int_time = 400000;

		msleep(int_time / 1000);

		pr_info("reading address chan->address 0x%lx\n", chan->address);
		ret = bu27008_chan_read_data(data, chan->address, val);
		if (!ret)
			ret = IIO_VAL_INT;
		bu27008_meas_dis(data);
		mutex_unlock(&data->mutex);

		iio_device_release_direct_mode(idev);

		return ret;
	}
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
	int ret, gain_sel, time_sel, i;

	mutex_lock(&data->mutex);

	ret = bu27008_get_int_time_sel(data, &time_sel);
	if (ret < 0)
		goto unlock_out;


	ret = iio_gts_find_gain_sel_for_scale_using_time(&data->gts, time_sel,
						val, val2 * 1000, &gain_sel);
	if (ret) {
		/* Could not support new scale with existing int-time */
		int new_time_sel;

		for (i = 0; i < data->gts.num_itime; i++) {
			new_time_sel = data->gts.itime_table[i].sel;
			ret = iio_gts_find_gain_sel_for_scale_using_time(
				&data->gts, new_time_sel, val, val2 * 1000,
				&gain_sel);
			if (!ret)
				break;
		}
		if (i == data->gts.num_itime) {
			dev_err(data->dev, "Can't support scale %u %u\n", val,
				val2);

			ret = -EINVAL;
			goto unlock_out;
		}

		ret = bu27008_set_int_time_sel(data, new_time_sel);
		if (ret)
			goto unlock_out;
	}


	ret = bu27008_write_gain_sel(data, gain_sel);

unlock_out:
	mutex_unlock(&data->mutex);

	return ret;
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
		ret = bu27008_try_set_int_time(data, val2);
		break;
	default:
		ret = -EINVAL;
		break;
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

static int bu27008_read_avail(struct iio_dev *idev,
			      struct iio_chan_spec const *chan, const int **vals,
			      int *type, int *length, long mask)
{
	struct bu27008_data *data = iio_priv(idev);

	switch (mask) {
	case IIO_CHAN_INFO_INT_TIME:
		return iio_gts_avail_times(&data->gts, vals, type, length);
	case IIO_CHAN_INFO_SCALE:
		return iio_gts_all_avail_scales(&data->gts, vals, type, length);
	default:
		return -EINVAL;
	}
}

static const struct iio_info bu27008_info = {
	.read_raw = &bu27008_read_raw,
	.write_raw = &bu27008_write_raw,
	.read_avail = &bu27008_read_avail,
	.validate_trigger	= bu27008_validate_trigger,
};

static int bu27008_chip_init(struct bu27008_data *data)
{
	int ret;

	/* Reset */
	ret = regmap_update_bits(data->regmap, BU27008_REG_SYSTEM_CONTROL,
			   BU27008_MASK_SW_RESET, BU27008_MASK_SW_RESET);
	if (ret)
		return dev_err_probe(data->dev, ret, "Sensor reset failed\n");

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

	if (data->trigger_enabled)
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
	struct bu27008_data *data;
	struct regmap *regmap;
	unsigned int part_id, reg;
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

	idev = devm_iio_device_alloc(dev, sizeof(*data));
	if (!idev)
		return -ENOMEM;

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get regulator\n");

	data = iio_priv(idev);

	ret = regmap_read(regmap, BU27008_REG_SYSTEM_CONTROL, &reg);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to access sensor\n");

	part_id = FIELD_GET(BU27008_MASK_PART_ID, reg);

	if (part_id != BU27008_ID)
		dev_warn(dev, "unknown device 0x%x\n", part_id);

	ret = devm_iio_init_iio_gts(dev, BU27008_SCALE_1X, 0, bu27008_gains,
				    ARRAY_SIZE(bu27008_gains), bu27008_itimes,
				    ARRAY_SIZE(bu27008_itimes), &data->gts);
	if (ret)
		return ret;

	mutex_init(&data->mutex);
	data->regmap = regmap;
	data->dev = dev;
	data->irq = i2c->irq;

	idev->channels = bu27008_channels;
	idev->num_channels = ARRAY_SIZE(bu27008_channels);
	idev->name = "bu27008";
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
		.name  = "bu27008",
		.of_match_table = bu27008_of_match,
	  },
	.probe_new    = bu27008_probe,
};
module_i2c_driver(bu27008_i2c_driver);

MODULE_DESCRIPTION("ROHM BU27008 colour sensor driver");
MODULE_AUTHOR("Matti Vaittinen <matti.vaittinen@fi.rohmeurope.com>");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_GTS_HELPER);
