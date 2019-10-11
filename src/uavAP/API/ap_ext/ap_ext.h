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
/*
 *
 */
#ifndef UAVAP_EXT_H__
#define UAVAP_EXT_H__

#include <stdio.h>
#include <stdlib.h>
#include <linux/types.h>
#include <stdint.h>
#include <unistd.h>

#include <iostream>
#include <fstream>

#define PWM_CHS 22
#define ADC_CHS 32
#define LOADC_CHS 2
#define NUM_SLINK_CHS (1)
#define PIC_BUFFER_SIZE (sizeof(struct pic_sample_t))
#define LINE_SEP_MAXLEN 10
#define MOTOR_CHS 1
#define SLINK_NUM_CMDS 11

#define AIRSPEED_ADC_CH 18
#define USE_AUTOPILOT

struct data_sample_t;
struct imu_sample_t;
struct pic_sample_t;
struct slink_sample_t;
struct output_settings_t;

/* Definition of a sample as acquired from sensors */
struct data_sample_t
{
	struct imu_sample_t * imu_sample;
	struct imu_sample_t * int_imu_sample;
	struct pic_sample_t * pic_sample;
	struct airs_sample_t * airs_sample;
	struct slink_sample_t * slink_sample;
	struct phidget_sample_t * phidget_sample;
};

#if 0
struct int_imu_sample_t {
    unsigned long valid_flags;
    unsigned long imu_pkt;

    union {
	struct {
	    double imu_euler_roll;
	    double imu_euler_pitch;
	    double imu_euler_yaw;
	};
	    double imu_euler[3];
    };

    union {
	struct {
	    double imu_quat_w;
	    double imu_quat_x;
	    double imu_quat_y;
	    double imu_quat_z;
	};
	    double imu_quat[4];
    };

    union {
	struct {
	    double imu_accel_x;
	    double imu_accel_y;
	    double imu_accel_z;
	};
	double imu_accel[3];
    };

    union {
	struct {
	    double imu_rot_x;
	    double imu_rot_y;
	    double imu_rot_z;
	};
	double imu_rot[3];
    };

    union {
	struct {
	    double imu_mag_x;
	    double imu_mag_y;
	    double imu_mag_z;
	};
	double imu_mag[3];
    };

};

#endif

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
			double imu_quat_w;
			double imu_quat_x;
			double imu_quat_y;
			double imu_quat_z;
		};
		double imu_quat[4];
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

struct pts_sample_t
{
	float press;
	float temp;
};

/* END of PIC-dependent structures */

struct pic_sample_t
{
	unsigned short adc_channels[ADC_CHS];
	struct pts_sample_t pts_sample;
	unsigned long pwm_channels[PWM_CHS];
	struct GPS_sample gps_sample;
	unsigned long pkt_nr;
	unsigned short user_action;
	unsigned short fw_version;
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
	} channels[NUM_SLINK_CHS];
};

struct phidget_sample_t
{
	double loadc_ch[LOADC_CHS];
};

struct airs_sample_t
{
	unsigned short airs;
#ifdef USE_AUTOPILOT
	double cal_airs;
#endif
	/* These fields should replace internal PIC measurements if an
	 * external probe is available */
	float press;
	float temp;
};

struct adc_ch_setting_t
{
	char enabled;
	char format;
	char range;
	char reserved;
	double min;
	double max;
	/* Filled at output format parsing time */
	double scale;
};

struct generic_field_setting_t
{
	char enabled;
	char format;
};

struct pwm_ch_setting_t
{
	char enabled;
	char format;
};

struct imu_field_setting_t
{
	char enabled;
	char format;
};

struct gps_field_setting_t
{
	char enabled;
	char format;
};

struct esc_field_setting_t
{
	char enabled;
	char format;
};

struct airs_field_setting_t
{
	char enabled;
	char format;
	char calibrated;
};

struct pts_field_setting_t
{
	char enabled;
	char format;
};

struct imu_setting_t
{
	imu_field_setting_t pkt;
	imu_field_setting_t euler;
	imu_field_setting_t accel;
	imu_field_setting_t temp;
	imu_field_setting_t press;
	imu_field_setting_t lat;
	imu_field_setting_t lon;
	imu_field_setting_t alt;
	imu_field_setting_t rot;
	imu_field_setting_t mag;
	imu_field_setting_t vel;
	imu_field_setting_t time_day;
	imu_field_setting_t time_hour;
	imu_field_setting_t time_minute;
	imu_field_setting_t time_month;
	imu_field_setting_t time_nano;
	imu_field_setting_t time_second;
	imu_field_setting_t time_year;
};

struct gps_setting_t
{
	gps_field_setting_t lat;
	gps_field_setting_t lon;
	gps_field_setting_t alt;
	gps_field_setting_t sep;
	gps_field_setting_t fix;
	gps_field_setting_t sats;
	gps_field_setting_t course_gnd;
	gps_field_setting_t speed_gnd;
	gps_field_setting_t speed_vert;
	gps_field_setting_t time_day;
	gps_field_setting_t time_hour;
	gps_field_setting_t time_minute;
	gps_field_setting_t time_month;
	gps_field_setting_t time_second;
	gps_field_setting_t time_year;
};

struct esc_setting_t
{
	esc_field_setting_t volt;
	esc_field_setting_t curr;
	esc_field_setting_t thr;
	esc_field_setting_t rpm;
	esc_field_setting_t bec_volt;
	esc_field_setting_t bec_curr;
};

struct airs_setting_t
{
	airs_field_setting_t airs;
};

struct pts_setting_t
{
	pts_field_setting_t press;
	pts_field_setting_t temp;
};

struct output_stats_t
{
	size_t total_bytes;
	size_t bytes_sample;
	size_t sample_count;
	size_t length;
};

struct run_requirements_t
{
	union
	{
		struct
		{
			unsigned long imu :1;
			unsigned long int_imu :1;
			unsigned long pic :1;
			unsigned long slink :1;
			unsigned long airs :1;
			unsigned long phidget :1;
		};
		unsigned long raw_mask;
	};

	unsigned int dac_val[2];
};

struct output_settings_t
{
	char enabled;
	char active;
	char done_cmd;
	ulong speed;
	char field_sep;
	char field_sep_char;
	char line_sep_char[LINE_SEP_MAXLEN];
	/* Not a DB field */
	ulong line_sep_char_len;
	char use_imu;
	int outfd;
	unsigned long log_id;
	struct adc_ch_setting_t adc_ch[ADC_CHS];
	struct pwm_ch_setting_t pwm_ch[PWM_CHS];
	struct imu_setting_t imu_setup;
	struct imu_setting_t int_imu_setup;
	struct gps_setting_t gps_setup;
	struct esc_setting_t esc_setup;
	struct airs_setting_t airs_setup;
	struct pts_setting_t pts_setup;
	struct output_stats_t stats;
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
EXTERNC int
ap_ext_ctrl(int* cmd);

#undef EXTERNC

class ApExtManager;

ApExtManager*
getApExtManager();

void
setConfigPath(const std::string& configPath);

#endif
