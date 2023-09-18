#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/string_helpers.h>
#include <linux/units.h>
#include <linux/delay.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#include <asm/unaligned.h>

#define LTR579_MAIN_CTRL 0x00
#define LTR579_PS_LED 0x01
#define LTR579_PS_PULSES 0x02
#define LTR579_PS_MEAS_RATE 0x03
#define LTR579_ALS_MEAS_RATE 0x04
#define LTR579_ALS_GAIN 0x05
#define LTR579_PART_ID 0x06
#define LTR579_MAIN_STATUS 0x07
#define LTR579_PS_DATA_0 0x08
#define LTR579_PS_DATA_1 0x09
#define LTR579_ALS_DATA_0 0x0d
#define LTR579_ALS_DATA_1 0x0e
#define LTR579_ALS_DATA_2 0x0f
#define LTR579_INT_CFG 0x19
#define LTR579_INT_PST 0x1a
#define LTR579_PS_THRES_UP_0 0x1B
#define LTR579_PS_THRES_UP_1 0x1C
#define LTR579_PS_THRES_LOW_0 0x1D
#define LTR579_PS_THRES_LOW_1 0x1E
#define LTR579_PS_CAN_0 0x1F
#define LTR579_PS_CAN_1 0x20
#define LTR579_ALS_THRES_UP_0 0x21
#define LTR579_ALS_THRES_UP_1 0x22
#define LTR579_ALS_THRES_UP_2 0x23
#define LTR579_ALS_THRES_LOW_0 0x24
#define LTR579_ALS_THRES_LOW_1 0x25
#define LTR579_ALS_THRES_LOW_2 0x26

#define LTR579_PS_ENABLE_MASK BIT(0)
#define LTR579_RESET_MASK BIT(4)
#define LTR579_ALS_DATA_STATUS BIT(3)
#define LTR579_ALS_ENABLE_MASK BIT(1)
#define LTR579_PS_OVERFLOW_MASK BIT(3)

#define LTR579_WIN_FAC 1

#define LTR_579_WHO_AM_I 0xb1

enum ltr579_fields {
	/* MAIN_CTRL */
	F_PS_ENABLE,
	F_ALS_ENABLE,
	F_SOFT_RESET,

	/* PS_LED */
	F_PS_LED_CURRENT,
	F_PS_LED_PULSE_MODULATION_FREQ,

	/* PS_PULSES */
	F_PS_NUM_OF_PULSES,

	/* PS_MEAS_RATE */
	F_PS_MEAS_RATE,
	F_PS_RESOLUTION,

	/* ALS_MEAS_RATE */
	F_ALS_MEAS_RATE,
	F_ALS_RESOLUTION,

	/* ALS_GAIN */
	F_ALS_GAIN,

	/* PART_ID */
	F_PART_NUM_ID,
	F_REVISION_ID,

	/* MAIN_STATUS */
	F_PS_DATA_STATUS,
	F_PS_INTERRUPT_STATUS,
	F_PS_LOGIC_SIGNAL_STATUS,
	F_ALS_DATA_STATUS,
	F_ALS_INTERRUPT_STATUS,
	F_POWER_ON_STATUS,

	/* PS_DATA_0 */
	F_PS_DATA_LOW,

	/* PS_DATA_1 */
	F_PS_DATA_HIGH,
	F_PS_OVERFLOW,

	/* ALS_DATA_0 */
	F_ALS_DATA_LOW,

	/* ALS_DATA_1 */
	F_ALS_DATA_MIDDLE,

	/* ALS_DATA_2 */
	F_ALS_DATA_HIGH,

	/* INT_CFG */
	F_PS_INTERRUPT_PIN_ENABLE,
	F_PS_OUTPUT_MODE,
	F_ALS_INTERRUPT_PIN_ENABLE,
	F_ALS_INTERRUPT_SELECT,

	/* INT_PST */
	F_PS_PERSIST,
	F_ALS_PERSIST,

	/* PS_THRES_UP_0 */
	F_PS_UPPER_THRESHOLD_LOW,

	/* PS_THRES_UP_1 */
	F_PS_UPPER_THRESHOLD_HIGH,

	/* PS_THRES_LOW_0 */
	F_PS_LOWER_THRESHOLD_LOW,

	/* PS_THRES_LOW_1 */
	F_PS_LOWER_THRESHOLD_HIGH,

	/* PS_CAN_0 */
	F_PS_CANCELLATION_LEVEL_LOW,

	/* PS_CAN_1 */
	F_PS_CANCELLATION_LEVEL_HIGH,

	/* ALS_THRES_UP_O */
	F_ALS_UPPER_THRESHOLD_LOW,
	/* ALS_THRES_UP_1 */
	F_ALS_UPPER_THRESHOLD_MIDDLE,
	/* ALS_THRES_UP_2 */
	F_ALS_UPPER_THRESHOLD_HIGH,

	/* ALS_THRES_LOW_O */
	F_ALS_LOWER_THRESHOLD_LOW,
	/* ALS_THRES_LOW_1 */
	F_ALS_LOWER_THRESHOLD_MIDDLE,
	/* ALS_THRES_LOW_2 */
	F_ALS_LOWER_THRESHOLD_HIGH,

	F_MAX_FIELDS,
};

static const struct reg_field ltr579_reg_fields[] = {
	/* MAIN_CTRL */
	[F_PS_ENABLE] = REG_FIELD(LTR579_MAIN_CTRL, 0, 0),
	[F_ALS_ENABLE] = REG_FIELD(LTR579_MAIN_CTRL, 1, 1),
	[F_SOFT_RESET] = REG_FIELD(LTR579_MAIN_CTRL, 4, 4),

	/* PS_LED */
	[F_PS_LED_CURRENT] = REG_FIELD(LTR579_PS_LED, 0, 2),
	[F_PS_LED_PULSE_MODULATION_FREQ] = REG_FIELD(LTR579_PS_LED, 4, 6),

	/* PS_PULSES */
	[F_PS_NUM_OF_PULSES] = REG_FIELD(LTR579_PS_PULSES, 0, 5),

	/* PS_MEAS_RATE */
	[F_PS_MEAS_RATE] = REG_FIELD(LTR579_PS_MEAS_RATE, 0, 2),
	[F_PS_RESOLUTION] = REG_FIELD(LTR579_PS_MEAS_RATE, 3, 4),

	/* ALS_MEAS_RATE */
	[F_ALS_MEAS_RATE] = REG_FIELD(LTR579_ALS_MEAS_RATE, 0, 2),
	[F_ALS_RESOLUTION] = REG_FIELD(LTR579_ALS_MEAS_RATE, 4, 6),

	/* ALS_GAIN */
	[F_ALS_GAIN] = REG_FIELD(LTR579_ALS_GAIN, 0, 2),

	/* PART_ID */
	[F_PART_NUM_ID] = REG_FIELD(LTR579_ALS_GAIN, 0, 3),
	[F_REVISION_ID] = REG_FIELD(LTR579_ALS_GAIN, 4, 7),

	/* MAIN_STATUS */
	[F_PS_DATA_STATUS] = REG_FIELD(LTR579_MAIN_STATUS, 0, 0),
	[F_PS_INTERRUPT_STATUS] = REG_FIELD(LTR579_MAIN_STATUS, 1, 1),
	[F_PS_LOGIC_SIGNAL_STATUS] = REG_FIELD(LTR579_MAIN_STATUS, 2, 2),
	[F_ALS_DATA_STATUS] = REG_FIELD(LTR579_MAIN_STATUS, 3, 3),
	[F_ALS_INTERRUPT_STATUS] = REG_FIELD(LTR579_MAIN_STATUS, 4, 4),
	[F_POWER_ON_STATUS] = REG_FIELD(LTR579_MAIN_STATUS, 5, 5),

	/* PS_DATA_0 */
	[F_PS_DATA_LOW] = REG_FIELD(LTR579_PS_DATA_0, 0, 7),

	/* PS_DATA_1 */
	[F_PS_DATA_HIGH] = REG_FIELD(LTR579_PS_DATA_1, 0, 2),
	[F_PS_OVERFLOW] = REG_FIELD(LTR579_PS_DATA_1, 3, 3),

	/* ALS_DATA_0 */
	[F_ALS_DATA_LOW] = REG_FIELD(LTR579_ALS_DATA_0, 0, 7),

	/* ALS_DATA_1 */
	[F_ALS_DATA_MIDDLE] = REG_FIELD(LTR579_ALS_DATA_1, 0, 7),

	/* ALS_DATA_2 */
	[F_ALS_DATA_HIGH] = REG_FIELD(LTR579_ALS_DATA_2, 0, 3),

	/* INT_CFG */
	[F_PS_INTERRUPT_PIN_ENABLE] = REG_FIELD(LTR579_INT_CFG, 0, 0),
	[F_PS_OUTPUT_MODE] = REG_FIELD(LTR579_INT_CFG, 1, 1),
	[F_ALS_INTERRUPT_PIN_ENABLE] = REG_FIELD(LTR579_INT_CFG, 2, 2),
	[F_ALS_INTERRUPT_SELECT] = REG_FIELD(LTR579_INT_CFG, 4, 5),

	/* INT_PST */
	[F_PS_PERSIST] = REG_FIELD(LTR579_INT_PST, 0, 3),
	[F_ALS_PERSIST] = REG_FIELD(LTR579_INT_PST, 4, 7),

	/* PS_THRES_UP_0 */
	[F_PS_UPPER_THRESHOLD_LOW] = REG_FIELD(LTR579_PS_THRES_UP_0, 0, 7),

	/* PS_THRES_UP_1 */
	[F_PS_UPPER_THRESHOLD_HIGH] = REG_FIELD(LTR579_PS_THRES_UP_1, 0, 2),

	/* PS_THRES_LOW_0 */
	[F_PS_LOWER_THRESHOLD_LOW] = REG_FIELD(LTR579_PS_THRES_LOW_0, 0, 7),

	/* PS_THRES_LOW_1 */
	[F_PS_LOWER_THRESHOLD_HIGH] = REG_FIELD(LTR579_PS_THRES_LOW_1, 0, 2),

	/* PS_CAN_0 */
	[F_PS_CANCELLATION_LEVEL_LOW] = REG_FIELD(LTR579_PS_CAN_0, 0, 7),

	/* PS_CAN_1 */
	[F_PS_CANCELLATION_LEVEL_HIGH] = REG_FIELD(LTR579_PS_CAN_1, 0, 2),

	/* ALS_THRES_UP_0 */
	[F_ALS_UPPER_THRESHOLD_LOW] = REG_FIELD(LTR579_ALS_THRES_UP_0, 0, 7),
	/* ALS_THRES_UP_1 */
	[F_ALS_UPPER_THRESHOLD_MIDDLE] = REG_FIELD(LTR579_ALS_THRES_UP_1, 0, 7),
	/* ALS_THRES_UP_2 */
	[F_ALS_UPPER_THRESHOLD_HIGH] = REG_FIELD(LTR579_ALS_THRES_UP_2, 0, 3),

	/* ALS_THRES_LOW_0 */
	[F_ALS_LOWER_THRESHOLD_LOW] = REG_FIELD(LTR579_ALS_THRES_LOW_0, 0, 7),
	/* ALS_THRES_LOW_1 */
	[F_ALS_LOWER_THRESHOLD_MIDDLE] =
		REG_FIELD(LTR579_ALS_THRES_LOW_1, 0, 7),
	/* ALS_THRES_LOW_2 */
	[F_ALS_LOWER_THRESHOLD_HIGH] = REG_FIELD(LTR579_ALS_THRES_LOW_2, 0, 3),
};

static const struct regmap_range ltr579_readable_registers[] = {
	regmap_reg_range(LTR579_MAIN_CTRL, LTR579_PS_DATA_1),
	regmap_reg_range(LTR579_ALS_DATA_0, LTR579_ALS_DATA_2),
	regmap_reg_range(LTR579_INT_CFG, LTR579_ALS_THRES_LOW_2),
};

static const struct regmap_access_table ltr579_readable_table = {
	.yes_ranges = ltr579_readable_registers,
	.n_yes_ranges = ARRAY_SIZE(ltr579_readable_registers),
};

static const struct regmap_range ltr579_writeable_registers[] = {
	regmap_reg_range(LTR579_MAIN_CTRL, LTR579_ALS_GAIN),
	regmap_reg_range(LTR579_INT_CFG, LTR579_ALS_THRES_LOW_2),
};

static const struct regmap_access_table ltr579_writeable_table = {
	.yes_ranges = ltr579_writeable_registers,
	.n_yes_ranges = ARRAY_SIZE(ltr579_writeable_registers),
};

static const struct regmap_range ltr579_volatile_registers[] = {
	regmap_reg_range(LTR579_MAIN_STATUS, LTR579_MAIN_STATUS),
	regmap_reg_range(LTR579_PS_DATA_0, LTR579_ALS_DATA_2),
};

static const struct regmap_access_table ltr579_volatile_table = {
	.yes_ranges = ltr579_volatile_registers,
	.n_yes_ranges = ARRAY_SIZE(ltr579_volatile_registers),
};

static const struct regmap_config ltr579_regmap_config = {
	.name = "ltr579",
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = LTR579_ALS_THRES_LOW_2,
	.wr_table = &ltr579_writeable_table,
	.rd_table = &ltr579_readable_table,
	.volatile_table = &ltr579_volatile_table,
	.cache_type = REGCACHE_RBTREE,
};

struct ltr579_als_resolution {
	u8 resolution; /* bits */
	int conversion_time; /* ms */
	int int_factor;
};

/* ms */
static const struct ltr579_als_resolution ltr579_als_resolution_table[] = {
	{
		.resolution = 20,
		.conversion_time = 400,
		.int_factor = 25,
	},
	{
		.resolution = 19,
		.conversion_time = 200,
		.int_factor = 50,
	},
	{
		.resolution = 18,
		.conversion_time = 100, /* default */
		.int_factor = 100,
	},
	{
		.resolution = 17,
		.conversion_time = 50,
		.int_factor = 200,
	},
	{
		.resolution = 16,
		.conversion_time = 25,
		.int_factor = 400,
	},
};

/**
 * Must be bigger than conversion time
 * ms
 */
static const int ltr579_als_measurement_rate_table[] = {
	25,  50,   100, /* default */
	500, 1000, 2000, 2000,
};

static const u8 ltr579_als_gain_range_table[] = {
	1, 3, /* default */
	6, 9, 18,
};

static const int ltr579_als_gain_range_external_table[][2] = {
	{ 1, 0 }, { 0, 333333 }, { 0, 166666 }, { 0, 111111 }, { 0, 55555 },
};

/* kHz */
static const int ltr579_ps_led_pulse_period_table[] = {
	-1, -1, -1, 60, /* default */
	70, 80, 90, 100,
};

/* microA */
static const int ltr579_ps_led_pulsed_current_level_table[] = {
	2500,	5000, 10000, 25000, 50000, 75000, 100000, /* default */
	125000,
};

/* bits */
static const u8 ltr579_ps_resolution_table[] = {
	8, /* default */
	9,
	10,
	11,
};

/**
 * Must be bigger than conversion time
 * micro seconds
 */
static const int ltr579_ps_measurement_rate_table[] = {
	-1,	6250,	12500, 25000, 50000, 100000, /* default */
	200000, 400000,
};

/**
 * struct ltr579_priv - LTR579 internal private state
 * @regs: Underlying I2C bus adapter used to abstract slave
 *        register accesses
 * @fields: Abstract objects for each registers fields access
 * @dev: Device handler associated with appropriate bus client
 * @lock: Protects ltr579 device state between setup and data access routines
 *        (power transitions, samp_freq/scale tune, retrieving data, etc)
 * @chip_name: Chip name in the format "ltr579-%02x" % partid
 */
struct ltr579_priv {
	struct regmap *regs;
	struct regmap_field *fields[F_MAX_FIELDS];

	struct device *dev;
	struct mutex lock;
};

typedef int (*read_function)(struct ltr579_priv *);

static const struct iio_chan_spec ltr579_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate =
			BIT(IIO_CHAN_INFO_RAW) | BIT(IIO_CHAN_INFO_PROCESSED) |
			BIT(IIO_CHAN_INFO_SCALE) | BIT(IIO_CHAN_INFO_INT_TIME) |
			BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
	{
		.type = IIO_PROXIMITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW) |
				      BIT(IIO_CHAN_INFO_SAMP_FREQ),
	},
};

static void ltr579_reset(struct iio_dev *indio_dev)
{
	struct ltr579_priv *data = iio_priv(indio_dev);

	/* reset sensor, chip fails to respond to this, so ignore any errors */
	regmap_field_write(data->fields[F_SOFT_RESET], 1);

	/* wait for oscillator (power up) */
	usleep_range(5000, 6000);
}

static int ltr579_read_ps_data(struct ltr579_priv *data)
{
	struct device *dev = data->dev;
	int ret;
	u8 buf[2];

	ret = regmap_bulk_read(data->regs, LTR579_PS_DATA_0, buf, sizeof(buf));
	if (ret) {
		dev_err(dev, "failed to read measurement data: %d\n", ret);
		return ret;
	}

	return get_unaligned_le16(&buf[0]);
}

static int ltr579_get_als_resolution(struct ltr579_priv *ltr579)
{
	struct device *dev = ltr579->dev;

	unsigned int resolution;
	int err;

	err = regmap_field_read(ltr579->fields[F_ALS_RESOLUTION], &resolution);

	if (err) {
		dev_err(dev, "failed to read ltr579 als resolution: %d\n", err);
		return err;
	}

	return resolution;
}

static int ltr579_get_als_gain(struct ltr579_priv *ltr579)
{
	struct device *dev = ltr579->dev;

	unsigned int gain;
	int err;

	err = regmap_field_read(ltr579->fields[F_ALS_GAIN], &gain);

	if (err) {
		dev_err(dev, "failed to read ltr579 als gain: %d\n", err);
		return err;
	}

	return gain;
}

static int ltr579_read_als_data(struct ltr579_priv *data)
{
	struct device *dev = data->dev;
	int ret;
	u8 buf[3];

	struct ltr579_als_resolution resolution =
		ltr579_als_resolution_table[ltr579_get_als_resolution(data)];
	msleep(resolution.conversion_time);
	ret = regmap_bulk_read(data->regs, LTR579_ALS_DATA_0, buf, sizeof(buf));
	if (ret) {
		dev_err(dev, "failed to read measurement data: %d\n", ret);
		return ret;
	}

	return get_unaligned_le24(&buf[0]);
}

static int ltr579_get_lux(struct ltr579_priv *data)
{
	int greendata, resolution_index, gain;
	u64 lux, div;

	gain = ltr579_get_als_gain(data);
	if (gain < 0)
		return gain;

	resolution_index = ltr579_get_als_resolution(data);
	if (resolution_index < 0)
		return resolution_index;

	greendata = ltr579_read_als_data(data);
	if (greendata < 0)
		return greendata;

	lux = greendata * 80 * LTR579_WIN_FAC;
	div = ltr579_als_gain_range_table[gain] *
	      ltr579_als_resolution_table[resolution_index].int_factor;

	return div_u64(lux, div);
}

static int ltr579_read_raw_template(struct ltr579_priv *data,
				    read_function read_function, u8 mask,
				    int *val)
{
	int ret;
	mutex_lock(&data->lock);
	/* enable sensor */
	ret = regmap_update_bits(data->regs, LTR579_MAIN_CTRL, mask, 1);
	/* wait for oscillator (power up) */
	usleep_range(5000, 6000);
	if (ret < 0) {
		dev_err(data->dev, "can't enable");
		mutex_unlock(&data->lock);
		return ret;
	}
	ret = read_function(data);
	if (ret < 0) {
		dev_err(data->dev, "can't read");
		mutex_unlock(&data->lock);
		return ret;
	} else {
		*val = ret;
	}
	/* enable sensor */
	ret = regmap_update_bits(data->regs, LTR579_MAIN_CTRL, mask, 0);
	/* wait for oscillator (shutdown) */
	usleep_range(5000, 6000);
	mutex_unlock(&data->lock);
	if (ret < 0) {
		dev_err(data->dev, "can't disable");
		return ret;
	}
	return IIO_VAL_INT;
}

static int ltr579_als_gain_read_raw(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan, int *val,
				    int *val2, long mask)
{
	int ret;
	struct ltr579_priv *data = iio_priv(indio_dev);

	ret = ltr579_get_als_gain(data);
	if (ret < 0)
		return ret;

	*val = ltr579_als_gain_range_external_table[ret][0];
	*val2 = ltr579_als_gain_range_external_table[ret][1];

	return IIO_VAL_INT_PLUS_MICRO;
}

static int
ltr579_als_integration_time_read_raw(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan, int *val,
				     int *val2, long mask)
{
	int ret;
	struct ltr579_priv *data = iio_priv(indio_dev);

	ret = ltr579_get_als_resolution(data);
	if (ret < 0)
		return ret;

	*val = ltr579_als_resolution_table[ret].conversion_time;

	return IIO_VAL_INT;
}

static int ltr579_ms_to_hz(int ms, int *val, int *val2)
{
	*val = 1000 / ms;
	*val2 = (1000000000 / ms) % 1000000;
	return IIO_VAL_INT_PLUS_MICRO;
}

static int ltr579_micros_to_hz(int micros, int *val, int *val2)
{
	*val = 1000000 / micros;
	*val2 = (1000000000 / (micros / 1000)) % 1000000;
	return IIO_VAL_INT_PLUS_MICRO;
}

static int
ltr579_als_measurement_rate_read_raw(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan, int *val,
				     int *val2, long mask)
{
	int ret, measurement_rate, measurement_rate_index;
	struct ltr579_priv *data = iio_priv(indio_dev);

	ret = regmap_field_read(data->fields[F_ALS_MEAS_RATE],
				&measurement_rate_index);
	if (ret) {
		dev_err(data->dev,
			"failed to read ltr579 als measurement rate: %d\n",
			ret);
		return ret;
	}

	measurement_rate =
		ltr579_als_measurement_rate_table[measurement_rate_index];

	return ltr579_ms_to_hz(measurement_rate, val, val2);
}

static int ltr579_ps_measurement_rate_read_raw(struct iio_dev *indio_dev,
					       struct iio_chan_spec const *chan,
					       int *val, int *val2, long mask)
{
	int ret, measurement_rate, measurement_rate_index;
	struct ltr579_priv *data = iio_priv(indio_dev);

	ret = regmap_field_read(data->fields[F_PS_MEAS_RATE],
				&measurement_rate_index);
	if (ret) {
		dev_err(data->dev,
			"failed to read ltr579 ps measurement rate: %d\n", ret);
		return ret;
	}

	measurement_rate =
		ltr579_ps_measurement_rate_table[measurement_rate_index];

	return ltr579_micros_to_hz(measurement_rate, val, val2);
}

static int ltr579_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan, int *val,
			   int *val2, long mask)
{
	struct ltr579_priv *data = iio_priv(indio_dev);

	switch (chan->type) {
	case IIO_LIGHT:
		switch (mask) {
		case IIO_CHAN_INFO_RAW:
			return ltr579_read_raw_template(data,
							&ltr579_read_als_data,
							LTR579_ALS_ENABLE_MASK,
							val);
		case IIO_CHAN_INFO_PROCESSED:
			return ltr579_read_raw_template(data, &ltr579_get_lux,
							LTR579_ALS_ENABLE_MASK,
							val);
		case IIO_CHAN_INFO_SCALE:
			return ltr579_als_gain_read_raw(indio_dev, chan, val,
							val2, mask);
		case IIO_CHAN_INFO_INT_TIME:
			return ltr579_als_integration_time_read_raw(
				indio_dev, chan, val, val2, mask);
		case IIO_CHAN_INFO_SAMP_FREQ:
			return ltr579_als_measurement_rate_read_raw(
				indio_dev, chan, val, val2, mask);
		default:
			return -EINVAL;
		}
	case IIO_PROXIMITY:
		switch (mask) {
		case IIO_CHAN_INFO_RAW:
			return ltr579_read_raw_template(data,
							&ltr579_read_ps_data,
							LTR579_PS_ENABLE_MASK,
							val);
		case IIO_CHAN_INFO_SAMP_FREQ:
			return ltr579_ps_measurement_rate_read_raw(
				indio_dev, chan, val, val2, mask);
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static int
ltr579_als_measurement_rate_write_raw(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan, int val,
				      int val2, long mask)
{
	int ret, measurement_rate, conversion_time;
	struct ltr579_priv *data = iio_priv(indio_dev);

	measurement_rate = 1000000 / (val * 1000 + val2 / 1000);

	mutex_lock(&data->lock);

	regmap_field_read(data->fields[F_ALS_RESOLUTION], &conversion_time);

	if (ltr579_als_resolution_table[conversion_time].conversion_time >
	    measurement_rate) {
		dev_err(data->dev,
			"Can't set measurement rate as %d ms because it lower than conversion time %d ms",
			measurement_rate,
			ltr579_als_resolution_table[conversion_time]
				.conversion_time);
		mutex_unlock(&data->lock);
		return -EINVAL;
	}

	for (int i = 0; i < ARRAY_SIZE(ltr579_als_measurement_rate_table);
	     i++) {
		if (measurement_rate == ltr579_als_measurement_rate_table[i]) {
			ret = regmap_field_write(data->fields[F_ALS_MEAS_RATE],
						 i);
			mutex_unlock(&data->lock);
			return ret;
		}
	}

	dev_err(data->dev, "Can't find selected sampling frequency");
	mutex_unlock(&data->lock);
	return -EINVAL;
}

static int ltr579_als_gain_write_raw(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan, int val,
				     int val2, long mask)
{
	int ret;
	struct ltr579_priv *data = iio_priv(indio_dev);

	mutex_lock(&data->lock);

	for (int i = 0; i < ARRAY_SIZE(ltr579_als_gain_range_external_table);
	     i++) {
		if (val == ltr579_als_gain_range_external_table[i][0] &&
		    val2 == ltr579_als_gain_range_external_table[i][1]) {
			ret = regmap_field_write(data->fields[F_ALS_GAIN], i);
			mutex_unlock(&data->lock);
			return ret;
		}
	}

	dev_err(data->dev, "Can't find selected scale");
	mutex_unlock(&data->lock);
	return -EINVAL;
}

static int
ltr579_als_integration_time_write_raw(struct iio_dev *indio_dev,
				      struct iio_chan_spec const *chan, int val,
				      int val2, long mask)
{
	int ret, i, measurement_rate, measurement_rate_index,
		integration_time = val;
	struct ltr579_priv *data = iio_priv(indio_dev);

	mutex_lock(&data->lock);

	regmap_field_read(data->fields[F_ALS_MEAS_RATE],
			  &measurement_rate_index);
	measurement_rate =
		ltr579_als_measurement_rate_table[measurement_rate_index];
	if (integration_time < measurement_rate) {
		dev_warn(
			data->dev,
			"Selected measurement rate(%d ms) is faster than integration time (%d ms), finding fastest compatible measurement rate",
			integration_time, measurement_rate);
		for (i = 0; i < ARRAY_SIZE(ltr579_als_measurement_rate_table);
		     i++) {
			if (ltr579_als_measurement_rate_table[i] >
			    integration_time) {
				regmap_field_write(
					data->fields[F_ALS_MEAS_RATE], i);
				dev_warn(data->dev,
					 "Selected measurement rate - %d ms",
					 ltr579_als_measurement_rate_table[i]);
				break;
			}
		}
		if (i < 0) {
			dev_warn(
				data->dev,
				"Compatible measurement rate not found, sensor will set it automatically if integration time correct");
		}
	}

	for (int i = 0; i < ARRAY_SIZE(ltr579_als_resolution_table); i++) {
		if (integration_time ==
		    ltr579_als_resolution_table[i].conversion_time) {
			ret = regmap_field_write(data->fields[F_ALS_RESOLUTION],
						 i);
			mutex_unlock(&data->lock);
			return ret;
		}
	}

	dev_err(data->dev, "Can't find selected integration time");
	mutex_unlock(&data->lock);
	return -EINVAL;
}

static int
ltr579_ps_measurement_rate_write_raw(struct iio_dev *indio_dev,
				     struct iio_chan_spec const *chan, int val,
				     int val2, long mask)
{
	int ret, measurement_rate;
	struct ltr579_priv *data = iio_priv(indio_dev);

	measurement_rate = 1000000000 / (val * 1000000 + val2) * 1000;

	mutex_lock(&data->lock);

	for (int i = 0; i < ARRAY_SIZE(ltr579_ps_measurement_rate_table); i++) {
		if (measurement_rate == ltr579_ps_measurement_rate_table[i]) {
			ret = regmap_field_write(data->fields[F_PS_MEAS_RATE],
						 i);
			mutex_unlock(&data->lock);
			return ret;
		}
	}

	dev_err(data->dev, "Can't find selected sampling frequency");
	mutex_unlock(&data->lock);
	return -EINVAL;
}

static int ltr579_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan, int val, int val2,
			    long mask)
{
	switch (chan->type) {
	case IIO_LIGHT:
		switch (mask) {
		case IIO_CHAN_INFO_SCALE:
			return ltr579_als_gain_write_raw(indio_dev, chan, val,
							 val2, mask);
		case IIO_CHAN_INFO_INT_TIME:
			return ltr579_als_integration_time_write_raw(
				indio_dev, chan, val, val2, mask);
		case IIO_CHAN_INFO_SAMP_FREQ:
			return ltr579_als_measurement_rate_write_raw(
				indio_dev, chan, val, val2, mask);
		default:
			return -EINVAL;
		}
	case IIO_PROXIMITY:
		switch (mask) {
		case IIO_CHAN_INFO_SAMP_FREQ:
			return ltr579_ps_measurement_rate_write_raw(
				indio_dev, chan, val, val2, mask);
		default:
			return -EINVAL;
		}
	default:
		return -EINVAL;
	}
}

static ssize_t ltr579_ps_get_resolution(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct ltr579_priv *data = iio_priv(dev_to_iio_dev(dev));
	int val, ret;

	ret = regmap_field_read(data->fields[F_PS_RESOLUTION], &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n", ltr579_ps_resolution_table[val]);
}

static ssize_t ltr579_ps_set_resolution(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct ltr579_priv *data = iio_priv(dev_to_iio_dev(dev));
	unsigned long val;
	int ret, i;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ltr579_ps_resolution_table); i++) {
		if (val == ltr579_ps_resolution_table[i]) {
			ret = regmap_field_write(data->fields[F_PS_RESOLUTION],
						 i);
			if (ret < 0)
				return ret;
			return len;
		}
	}

	return -EINVAL;
}

static ssize_t ltr579_ps_get_led_pulse_period(struct device *dev,
					      struct device_attribute *attr,
					      char *buf)
{
	struct ltr579_priv *data = iio_priv(dev_to_iio_dev(dev));
	int val, ret;

	ret = regmap_field_read(data->fields[F_PS_LED_PULSE_MODULATION_FREQ],
				&val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n",
			  ltr579_ps_led_pulse_period_table[val] * 1000);
}

static ssize_t ltr579_ps_set_led_pulse_period(struct device *dev,
					      struct device_attribute *attr,
					      const char *buf, size_t len)
{
	struct ltr579_priv *data = iio_priv(dev_to_iio_dev(dev));
	unsigned long val;
	int ret, i;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ltr579_ps_led_pulse_period_table); i++) {
		if (val / 1000 == ltr579_ps_led_pulse_period_table[i]) {
			ret = regmap_field_write(
				data->fields[F_PS_LED_PULSE_MODULATION_FREQ],
				i);
			if (ret < 0)
				return ret;
			return len;
		}
	}

	return -EINVAL;
}

static ssize_t ltr579_ps_get_led_pulses(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct ltr579_priv *data = iio_priv(dev_to_iio_dev(dev));
	int val, ret;

	ret = regmap_field_read(data->fields[F_PS_NUM_OF_PULSES], &val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%d\n", val);
}

static ssize_t ltr579_ps_set_led_pulses(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t len)
{
	struct ltr579_priv *data = iio_priv(dev_to_iio_dev(dev));
	unsigned long val;
	int ret;

	ret = kstrtoul(buf, 10, &val);
	if (ret)
		return -EINVAL;

	if (val > 32 || val < 0)
		return -EINVAL;

	ret = regmap_field_write(data->fields[F_PS_NUM_OF_PULSES], val);
	if (ret < 0)
		return ret;

	return len;
}

static const char *ltr579_ps_get_led_pulsed_current_level_external[] = {
	"0.0025", "0.005", "0.01", "0.025", "0.05", "0.075", "0.1", "0.125",
};

static ssize_t
ltr579_ps_get_led_pulsed_current_level(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct ltr579_priv *data = iio_priv(dev_to_iio_dev(dev));
	int val, ret;

	ret = regmap_field_read(data->fields[F_PS_LED_PULSE_MODULATION_FREQ],
				&val);
	if (ret)
		return ret;

	return sysfs_emit(buf, "%s\n",
			  ltr579_ps_get_led_pulsed_current_level_external[val]);
}

static ssize_t
ltr579_ps_set_led_pulsed_current_level(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct ltr579_priv *data = iio_priv(dev_to_iio_dev(dev));
	int ret, i;

	for (i = 0;
	     i < ARRAY_SIZE(ltr579_ps_get_led_pulsed_current_level_external);
	     i++) {
		if (strcmp(buf,
			   ltr579_ps_get_led_pulsed_current_level_external[i])) {
			ret = regmap_field_write(data->fields[F_PS_LED_CURRENT],
						 i);
			if (ret < 0)
				return ret;
			return len;
		}
	}

	return -EINVAL;
}

static IIO_DEVICE_ATTR(in_proximity_resolution, S_IRUGO | S_IWUSR,
		       ltr579_ps_get_resolution, ltr579_ps_set_resolution, 0);

static IIO_DEVICE_ATTR(in_proximity_led_pulse_period, S_IRUGO | S_IWUSR,
		       ltr579_ps_get_led_pulse_period,
		       ltr579_ps_set_led_pulse_period, 0);

static IIO_DEVICE_ATTR(in_proximity_led_pulses, S_IRUGO | S_IWUSR,
		       ltr579_ps_get_led_pulses, ltr579_ps_set_led_pulses, 0);

static IIO_DEVICE_ATTR(in_proximity_led_pulsed_current_level, S_IRUGO | S_IWUSR,
		       ltr579_ps_get_led_pulsed_current_level,
		       ltr579_ps_set_led_pulsed_current_level, 0);

static IIO_CONST_ATTR(in_illuminance_scale_available,
		      "1 0.333333 0.166666 0.111111 0.055555");

static IIO_CONST_ATTR(in_illuminance_integration_time_available,
		      "400 200 100 50 25");

static IIO_CONST_ATTR(in_illuminance_sampling_frequency_available,
		      "40 20 10 2 1 0.5");

static IIO_CONST_ATTR(in_proximity_led_pulse_period_available,
		      "60000 70000 80000 90000 100000");

static IIO_CONST_ATTR(in_proximity_led_pulsed_current_level_available,
		      "0.0025 0.005 0.01 0.025 0.05 0.075 0.1 0.125");

static IIO_CONST_ATTR(
	in_proximity_led_pulses_available,
	"0 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32");

static IIO_CONST_ATTR(in_proximity_resolution_available, "8 9 10 11");

static IIO_CONST_ATTR(in_proximity_sampling_frequency_available,
		      "160 80 40 20 10 5 2.5");

static struct attribute *ltr579_attrs[] = {
	&iio_const_attr_in_illuminance_scale_available.dev_attr.attr,
	&iio_const_attr_in_illuminance_integration_time_available.dev_attr.attr,
	&iio_const_attr_in_illuminance_sampling_frequency_available.dev_attr
		 .attr,
	&iio_const_attr_in_proximity_led_pulse_period_available.dev_attr.attr,
	&iio_dev_attr_in_proximity_led_pulse_period.dev_attr.attr,
	&iio_const_attr_in_proximity_led_pulsed_current_level_available.dev_attr
		 .attr,
	&iio_dev_attr_in_proximity_led_pulsed_current_level.dev_attr.attr,
	&iio_const_attr_in_proximity_led_pulses_available.dev_attr.attr,
	&iio_dev_attr_in_proximity_led_pulses.dev_attr.attr,
	&iio_const_attr_in_proximity_resolution_available.dev_attr.attr,
	&iio_dev_attr_in_proximity_resolution.dev_attr.attr,
	&iio_const_attr_in_proximity_sampling_frequency_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ltr579_attr_group = {
	.attrs = ltr579_attrs,
};

static const struct iio_info ltr579_info = {
	.read_raw = ltr579_read_raw,
	.write_raw = ltr579_write_raw,
	.attrs = &ltr579_attr_group,
};

static int ltr579_regmap_init(struct ltr579_priv *ltr579)
{
	struct regmap_field **fields = ltr579->fields;
	struct device *dev = ltr579->dev;
	struct i2c_client *i2c = to_i2c_client(dev);
	struct regmap *regmap;
	int i;

	regmap = devm_regmap_init_i2c(i2c, &ltr579_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to register i2c regmap\n");

	ltr579->regs = regmap;

	for (i = 0; i < F_MAX_FIELDS; i++) {
		fields[i] = devm_regmap_field_alloc(dev, ltr579->regs,
						    ltr579_reg_fields[i]);
		if (IS_ERR(ltr579->fields[i]))
			return dev_err_probe(dev, PTR_ERR(ltr579->fields[i]),
					     "can't alloc field[%d]\n", i);
	}

	return 0;
}

static int ltr579_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ltr579_priv *ltr579;
	struct iio_dev *indio_dev;
	int err;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*ltr579));
	if (!indio_dev)
		return dev_err_probe(dev, -ENOMEM,
				     "IIO device allocation failed\n");

	indio_dev->info = &ltr579_info;
	indio_dev->name = "ltr579";
	indio_dev->channels = ltr579_channels;
	indio_dev->num_channels = ARRAY_SIZE(ltr579_channels);
	indio_dev->modes = INDIO_DIRECT_MODE;

	ltr579 = iio_priv(indio_dev);
	ltr579->dev = dev;
	i2c_set_clientdata(client, indio_dev);

	err = devm_regulator_get_enable(dev, "vdd");
	if (err)
		return dev_err_probe(dev, err, "can't get vdd supply\n");

	err = ltr579_regmap_init(ltr579);
	if (err)
		return err;
	mutex_init(&ltr579->lock);

	ltr579_reset(indio_dev);

	return devm_iio_device_register(dev, indio_dev);
}

static const struct i2c_device_id ltr579_id[] = {
	{ "ltr579" },
	{},
};

MODULE_DEVICE_TABLE(i2c, ltr579_id);

static const struct of_device_id ltr579_of_match[] = {
	{ .compatible = "liteon,ltr579" },
	{},
};

MODULE_DEVICE_TABLE(of, ltr579_of_match);

static struct i2c_driver ltr579_driver = {
    .driver = {
        .name = "ltr579",
        .of_match_table = ltr579_of_match,
    },
    .probe = ltr579_probe,
    .id_table = ltr579_id,
};
module_i2c_driver(ltr579_driver);

MODULE_AUTHOR("Eugene Lepshy <fekz115@gmail.com>");
MODULE_DESCRIPTION("ltr579 ambient light and proximity sensor driver");
MODULE_LICENSE("GPL");
