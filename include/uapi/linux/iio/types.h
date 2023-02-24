/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/* industrial I/O data types needed both in and out of kernel
 *
 * Copyright (c) 2008 Jonathan Cameron
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef _UAPI_IIO_TYPES_H_
#define _UAPI_IIO_TYPES_H_

/**
 * enum iio_chan_type - Type of data transferred via IIO channel.
 *
 * The 'main' type of data transferred via channel. Please note that most
 * devices also need to specify a more accurate 'sub category'. See the
 * enum iio_modifier for this. (For example, IIO_ACCEL channel often needs to
 * specify the direction. IIO_CONCENTRATION specifies the type of substance
 * it measures etc).
 *
 * These reflect the units of the measurement via processed or unit after
 * application of scale and offset. See the enum iio_chan_info_enum for
 * scale and offset.
 *
 * Please find the detailed documentation for reported values from the
 * Documentation/ABI/testing/sysfs-bus-iio.
 *
 * @IIO_ACCEL:		Acceleration, m/s^2
 *			Doc keyword: in_accel_x_raw
 *
 * @IIO_ACTIVITY:	Activity state. For example a pedometer signaling
 *			jogging, walking or staying still.
 *			Doc keyword: in_activity_still_thresh_rising_en
 *
 * @IIO_ALTVOLTAGE:	Peak to peak voltage, millivolts
 *
 * @IIO_ANGL:		Angle of rotation, radians.
 *			Doc keyword: in_angl_raw
 *
 * @IIO_ANGL_VEL:	Angular velocity, rad/s
 *			Doc keyword: in_anglvel_x_raw
 *
 * @IIO_CAPACITANCE:	Capacitance, nanofarads.
 *			Doc keyword: in_capacitanceY_raw
 *
 * @IIO_CCT:		Correlated color temperature, Kelvins
 *
 * @IIO_CURRENT:	Current, milliamps
 *			Doc keyword: in_currentY_raw
 *
 * @IIO_CONCENTRATION:	Reading of a substance, percents. Used for example by
 *			devices measuring amount of CO2, O2, ethanol...
 *			Doc keyword: in_concentration_raw
 *
 * @IIO_COUNT:		Deprecated, please use counter subsystem.
 *
 * @IIO_DISTANCE:	Distance in meters. Typically used to report measured
 *			distance to an object or the distance covered by the
 *			user
 *			Doc keyword: in_distance_input
 *
 * @IIO_ELECTRICALCONDUCTIVITY: electric conductivity, siemens per meter
 *			Doc keyword: in_electricalconductivity_raw
 *
 * @IIO_ENERGY:		Energy in Joules. Typically reported by a device
 *			measuring energy burnt by the user.
 *			Doc keyword: in_energy_input
 *
 * @IIO_GRAVITY:	Gravity, m/s^2
 *			Doc keyword: in_gravity_x_raw
 *
 * @IIO_HUMIDITYRELATIVE: Relative humidity, percents
 *			Doc keyword: in_humidityrelative_raw
 *
 * @IIO_INCLI:		Inclination, degrees
 *			Doc keyword: in_incli_x_raw
 *
 * @IIO_INDEX:		Deprecated, please use Counter subsystem
 *
 * @IIO_INTENSITY:	Unitless intensity.
 *			Doc keyword: in_intensityY_raw
 *
 * @IIO_LIGHT:		Visible light intensity, lux
 *			Doc keyword: in_illuminance_raw
 *
 * @IIO_MAGN:		Magnetic field, Gauss.
 *			Doc keyword: in_magn_x_raw
 *
 * @IIO_MASSCONCENTRATION: Mass concentration, ug / m3
 *			Doc keyword: in_massconcentration_pm1_input
 *
 * @IIO_PH:		pH reading, negative base-10 logarithm of hydrodium
 *			ions in a litre of water
 *			Doc keyword: in_ph_raw
 *
 * @IIO_PHASE:		Phase difference, radians
 *			Doc keyword: in_phaseY_raw
 *
 * @IIO_POSITIONRELATIVE: Relative position.
 *			Doc keyword: in_positionrelative_x_raw
 *
 * @IIO_POWER:		Power, milliwatts
 *			Doc keyword: in_powerY_raw
 *
 * @IIO_PRESSURE:	Pressure, kilopascal
 *			Doc keyword: in_pressureY_raw
 *
 * @IIO_RESISTANCE:	Resistance, ohms
 *			Doc keyword: in_resistance_raw
 *
 * @IIO_ROT:		Euler angles, deg
 *			Doc keyword: in_rot_yaw_raw
 *
 * @IIO_STEPS:		Steps taken by the user
 *			Doc keyword: in_steps_input
 *
 * @IIO_TEMP:		Temperature, milli degrees Celsius
 *			Doc keyword: in_temp_raw
 *
 * @IIO_UVINDEX:	UV light intensity index
 *			Doc keyword: in_uvindex_input
 *
 * @IIO_VELOCITY:	Current speed (norm or magnitude of the velocity
 *			vector), m/s
 *			Doc keyword: in_velocity_sqrt(x^2+y^2+z^2)_input
 *
 * @IIO_VOLTAGE:	Voltage, millivolts
 *			Doc keyword: in_voltageY_raw
 */
enum iio_chan_type {
	IIO_VOLTAGE,
	IIO_CURRENT,
	IIO_POWER,
	IIO_ACCEL,
	IIO_ANGL_VEL,
	IIO_MAGN,
	IIO_LIGHT,
	IIO_INTENSITY,
	IIO_PROXIMITY,
	IIO_TEMP,
	IIO_INCLI,
	IIO_ROT,
	IIO_ANGL,
	IIO_TIMESTAMP,
	IIO_CAPACITANCE,
	IIO_ALTVOLTAGE,
	IIO_CCT,
	IIO_PRESSURE,
	IIO_HUMIDITYRELATIVE,
	IIO_ACTIVITY,
	IIO_STEPS,
	IIO_ENERGY,
	IIO_DISTANCE,
	IIO_VELOCITY,
	IIO_CONCENTRATION,
	IIO_RESISTANCE,
	IIO_PH,
	IIO_UVINDEX,
	IIO_ELECTRICALCONDUCTIVITY,
	IIO_COUNT,
	IIO_INDEX,
	IIO_GRAVITY,
	IIO_POSITIONRELATIVE,
	IIO_PHASE,
	IIO_MASSCONCENTRATION,
};

/**
 * enum iio_modifier - accurate class for channel data
 *
 * @IIO_MOD_<X,Y,Z>:	Value represents <X,Y,Z>-axis data.
 *			Typically used by channels of type:
 *			IIO_ACCEL, IIO_TEMP, IIO_GRAVITY, IIO_POSITIONRELATIVE,
 *			IIO_ANGL_VEL, IIO_INCLI, IIO_MAGN
 * @IIO_MOD_LIGHT_BOTH:	Value contains visible and infrared light components
 * @IIO_MOD_LIGHT_IR:	Value represents infrared radiation
 * @IIO_MOD_LIGHT_<RED, GREEN, BLUE>:
 *			Value represents visible <red, green, blue>  light
 * @IIO_MOD_LIGHT_CLEAR:	Value represents all visible light frequencies
 *
 * Please find the detailed documentation for reported values from the
 * Documentation/ABI/testing/sysfs-bus-iio.
 */
enum iio_modifier {
	IIO_NO_MOD,
	IIO_MOD_X,
	IIO_MOD_Y,
	IIO_MOD_Z,
	IIO_MOD_X_AND_Y,
	IIO_MOD_X_AND_Z,
	IIO_MOD_Y_AND_Z,
	IIO_MOD_X_AND_Y_AND_Z,
	IIO_MOD_X_OR_Y,
	IIO_MOD_X_OR_Z,
	IIO_MOD_Y_OR_Z,
	IIO_MOD_X_OR_Y_OR_Z,
	IIO_MOD_LIGHT_BOTH,
	IIO_MOD_LIGHT_IR,
	IIO_MOD_ROOT_SUM_SQUARED_X_Y,
	IIO_MOD_SUM_SQUARED_X_Y_Z,
	IIO_MOD_LIGHT_CLEAR,
	IIO_MOD_LIGHT_RED,
	IIO_MOD_LIGHT_GREEN,
	IIO_MOD_LIGHT_BLUE,
	IIO_MOD_QUATERNION,
	IIO_MOD_TEMP_AMBIENT,
	IIO_MOD_TEMP_OBJECT,
	IIO_MOD_NORTH_MAGN,
	IIO_MOD_NORTH_TRUE,
	IIO_MOD_NORTH_MAGN_TILT_COMP,
	IIO_MOD_NORTH_TRUE_TILT_COMP,
	IIO_MOD_RUNNING,
	IIO_MOD_JOGGING,
	IIO_MOD_WALKING,
	IIO_MOD_STILL,
	IIO_MOD_ROOT_SUM_SQUARED_X_Y_Z,
	IIO_MOD_I,
	IIO_MOD_Q,
	IIO_MOD_CO2,
	IIO_MOD_VOC,
	IIO_MOD_LIGHT_UV,
	IIO_MOD_LIGHT_DUV,
	IIO_MOD_PM1,
	IIO_MOD_PM2P5,
	IIO_MOD_PM4,
	IIO_MOD_PM10,
	IIO_MOD_ETHANOL,
	IIO_MOD_H2,
	IIO_MOD_O2,
	IIO_MOD_LINEAR_X,
	IIO_MOD_LINEAR_Y,
	IIO_MOD_LINEAR_Z,
	IIO_MOD_PITCH,
	IIO_MOD_YAW,
	IIO_MOD_ROLL,
};

enum iio_event_type {
	IIO_EV_TYPE_THRESH,
	IIO_EV_TYPE_MAG,
	IIO_EV_TYPE_ROC,
	IIO_EV_TYPE_THRESH_ADAPTIVE,
	IIO_EV_TYPE_MAG_ADAPTIVE,
	IIO_EV_TYPE_CHANGE,
	IIO_EV_TYPE_MAG_REFERENCED,
	IIO_EV_TYPE_GESTURE,
};

enum iio_event_direction {
	IIO_EV_DIR_EITHER,
	IIO_EV_DIR_RISING,
	IIO_EV_DIR_FALLING,
	IIO_EV_DIR_NONE,
	IIO_EV_DIR_SINGLETAP,
	IIO_EV_DIR_DOUBLETAP,
};

#endif /* _UAPI_IIO_TYPES_H_ */
