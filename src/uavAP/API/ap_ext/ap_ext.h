////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2018 University of Illinois Board of Trustees
//
// This file is part of uavAP.
//
// uavAP is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// uavAP is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
////////////////////////////////////////////////////////////////////////////////
#ifndef __AP_EXT_H__
#define __AP_EXT_H__

#include <stdint.h>
#include <cstdint>

#define PWM_CHS 22
#define ADC_CHS 32
#define LOADC_CHS 2
#define MOTOR_CHS 1
#define SLINK_NUM_CMDS 11

#define AIRSPEED_ADC_CH 18

struct data_sample_t;
struct imu_sample_t;
struct pic_sample_t;
struct slink_sample_t;
struct output_settings_t;

// NOTE these may need extra double inclusion guard in order for AlVolo to compile

/* Definition of a sample as acquired from sensors */
struct data_sample_t
{
	struct imu_sample_t * imu_sample;
	struct int_imu_sample_t * int_imu_sample;
	struct pic_sample_t * pic_sample;
	struct rpm_sample_t * rpm_sample;
	struct slink_sample_t * slink_sample;
	struct phidget_sample_t * phidget_sample;
};

struct int_imu_sample_t
{
	unsigned long valid_flags;
	unsigned long imu_pkt;

	union
	{
		struct
		{
			double imu_euler_roll;
			double imu_euler_pitch;
			double imu_euler_yaw;
		};
		double imu_euler[3];
	};

	union
	{
		struct
		{
			double imu_accel_x;
			double imu_accel_y;
			double imu_accel_z;
		};
		double imu_accel[3];
	};

	union
	{
		struct
		{
			double imu_rot_x;
			double imu_rot_y;
			double imu_rot_z;
		};
		double imu_rot[3];
	};

	union
	{
		struct
		{
			double imu_mag_x;
			double imu_mag_y;
			double imu_mag_z;
		};
		double imu_mag[3];
	};

};

struct imu_sample_t
{
	unsigned long valid_flags;
	unsigned long imu_pkt;

	union
	{
		struct
		{
			double imu_euler_roll;
			double imu_euler_pitch;
			double imu_euler_yaw;
		};
		double imu_euler[3];
	};

	union
	{
		struct
		{
			double imu_accel_x;
			double imu_accel_y;
			double imu_accel_z;
		};
		double imu_accel[3];
	};

	double imu_temp;

	double imu_press;

	double imu_lat;
	double imu_lon;

	double imu_alt;

	union
	{
		struct
		{
			double imu_rot_x;
			double imu_rot_y;
			double imu_rot_z;
		};
		double imu_rot[3];
	};

	union
	{
		struct
		{
			double imu_mag_x;
			double imu_mag_y;
			double imu_mag_z;
		};
		double imu_mag[3];
	};

	union
	{
		struct
		{
			double imu_vel_x;
			double imu_vel_y;
			double imu_vel_z;
		};
		double imu_vel[3];
	};

	uint8_t imu_time_day;
	uint8_t imu_time_hour;
	uint8_t imu_time_minute;
	uint8_t imu_time_month;
	uint32_t imu_time_nano;
	uint8_t imu_time_second;
	uint16_t imu_time_year;

};

/* These definitions should match what defined inside uart_comm.h in
 * the PIC firmware */
struct PUBX_POS_fields
{
	char hour;
	char minute;
	/* This fields contains the fix and 
	 * checksum_valid flags*/
	char flags;
	char satellites;
	float second;
	float latitude;
	float longitude;
	float msl_altitude;
	float geoid_sep;
	char nav_status[2];
	float horiz_accuracy;
	float vert_accuracy;
	float course_gnd;
	float speed_gnd_kh;
	float vert_velocity;
};

struct PUBX_TIME_fields
{
	char hour;
	char minute;
	char day;
	char month;
	char year;
	char flags;
	float second;
};

struct GGA_fields
{
	char hour;
	char minute;
	/* This fields contains the fix and 
	 * checksum_valid flags*/
	char flags;
	char satellites;
	float second;
	float latitude;
	float longitude;
	float msl_altitude;
	float geoid_sep;
};

struct VTG_fields
{
	float course_gnd;
	float speed_gnd_kn;
	float speed_gnd_kh;
	char referece;
	char mode;
	char flags;
};

struct GPS_sample
{
	struct PUBX_POS_fields position;
	struct PUBX_TIME_fields time;
};

/* END of PIC-dependent structures */

struct pic_sample_t
{
	unsigned short adc_channels[ADC_CHS];
	unsigned long pwm_channels[PWM_CHS];
	struct GPS_sample gps_sample;
};

struct slink_sample_t
{
	union
	{
		struct
		{
			double volt;
			double ripple_volt;
			double current;
			double throttle;
			double out_power;
			double rpm;
			double bec_volt;
			double bec_current;
			double temp;
			double raw_ntc_temp;
			double raw_lin_temp;
		};
		double val[SLINK_NUM_CMDS];
	};
};

struct phidget_sample_t
{
	double loadc_ch[LOADC_CHS];
};

struct rpm_sample_t
{
	unsigned long motor_ch[MOTOR_CHS];
};

 #ifdef __cplusplus
 #define EXTERNC extern "C"
 #else
 #define EXTERNC
 #endif

 EXTERNC int
ap_ext_setup();
 EXTERNC int
ap_ext_sense(const struct data_sample_t * sample);
 EXTERNC int
ap_ext_actuate(unsigned long * pwm, unsigned int num_channels);
EXTERNC int
ap_ext_teardown();


 #undef EXTERNC

class ApExtManager;

ApExtManager*
getApExtManager();


#endif
