
/*
 * ApExtManager.cpp
 *
 *  Created on: Aug 31, 2017
 *      Author: mircot
 */
#include "uavAP/API/ap_ext/ApExtManager.h"
#include "uavAP/API/ap_ext/LinearSensorManager.h"
#include "uavAP/API/ap_ext/latLongToUTM.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/Core/DataHandling/DataHandling.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include <cmath>
#include <iostream>
#include <boost/thread/thread_time.hpp>

ApExtManager::ApExtManager() :
gpsSampleTimestamp_()
{
}


bool
ApExtManager::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			if (!checkIsSet<AggregatableAutopilotAPI>())
			{
				CPSLOG_ERROR << "Missing deps";
				return true;
			}
			if (!channelMixing_.configure(params.channelMixing()))
			{
				CPSLOG_ERROR << "Channel mixing configuration failed";
				return true;
			}
			airspeedFilter_.setParams(params.airspeedFilter());

			AdvancedControl advanced;
			ControllerOutput control;
			// Initialize last control and advanced control
			onAdvancedControl(advanced);
			onControllerOutput(control);
			break;
		}
		case RunStage::NORMAL:
		{
			auto aggAPI = get<AggregatableAutopilotAPI>();
			aggAPI->subscribeOnControllerOut([this](const auto& controllerOut)
											 { this->onControllerOutput(controllerOut); });
			aggAPI->subscribeOnAdvancedControl([this](const auto& advanced)
											   { this->onAdvancedControl(advanced); });

			if (auto dh = get<DataHandling>())
			{
				if (auto lsm = get<LinearSensorManager>())
				{
					dh->addStatusFunction<std::map<std::string, FloatingType>>([this]()
																				{ return this->getMiscValues(); },
																				Content::MISC_VALUES);
				}
			}

			break;
		}
		default:
			break;
	}
	return false;
}


int
ApExtManager::ap_sense(const data_sample_t* sample)
{
	SensorData sens;
	ServoData servo;
	PowerData power;

	//************************
	// IMU Data
	//************************
	Vector3 acceleration;
	Vector3 angularRate;
	Vector3 euler;
	Eigen::Quaterniond attitude;
	uint32_t packetNumber = 0;
	imu_sample_t* imuSample = nullptr;

	angularRate << 0.0, 0.0, 0.0;

	if (params.internalImu())
	{
		imuSample = sample->int_imu_sample;
	}
	else
	{
		imuSample = sample->imu_sample;
	}

	if (!imuSample)
	{
		CPSLOG_ERROR << "Cannot read imu sample from imu.";
	}
	else
	{
		//Attitude
		if (params.useEuler())
		{
			euler.x() = imuSample->imu_euler_roll;
			euler.y() = imuSample->imu_euler_pitch;
			euler.z() = imuSample->imu_euler_yaw;
		}
		else
		{
			attitude.w() = imuSample->imu_quat_w;
			attitude.x() = imuSample->imu_quat_x;
			attitude.y() = imuSample->imu_quat_y;
			attitude.z() = imuSample->imu_quat_z;
		}

		//Acceleration
		acceleration[0] = imuSample->imu_accel_x;
		acceleration[1] = imuSample->imu_accel_y;
		acceleration[2] = imuSample->imu_accel_z;

		//Rotation Rate
		angularRate[0] = imuSample->imu_rot_x;
		angularRate[1] = imuSample->imu_rot_y;
		angularRate[2] = imuSample->imu_rot_z;

		packetNumber = imuSample->imu_pkt;

		if (params.internalImu())
		{
			sens.timestamp = Clock::now();
		}
		else
		{
			try
			{
				boost::gregorian::date date(imuSample->imu_time_year, imuSample->imu_time_month,
											imuSample->imu_time_day);

				boost::posix_time::ptime time(date, boost::posix_time::milliseconds(0));

				auto dur = time
						   - boost::posix_time::ptime(
						boost::posix_time::special_values::min_date_time);

				Nanoseconds duration = Hours(imuSample->imu_time_hour)
									+ Minutes(imuSample->imu_time_minute) + Seconds(imuSample->imu_time_second)
									+ Nanoseconds(imuSample->imu_time_nano);
				sens.timestamp = TimePoint(std::chrono::duration_cast<TimePoint::duration>(
					duration + Nanoseconds(dur.total_nanoseconds())));
			} catch (std::out_of_range& err)
			{
				CPSLOG_DEBUG << "Time is not valid. " << err.what();
				sens.timestamp = Clock::now();
			}
		}
	}

	//Rotation Offset
//	if (params.rotationOffset())
//	{
//		//Rotation is offset. Use quaternions instead of eulers to read values.
//		auto rotationInverse = rotationOffset_->inverse();
//		attitude = (attitude * rotationInverse).normalized();
//
//		Eigen::Quaterniond acc;
//		acc.w() = 0;
//		acc.vec() = acceleration;
//		acceleration = (*rotationOffset_ * acc * rotationInverse).vec();
//
//		Eigen::Quaterniond angRate;
//		angRate.w() = 0;
//		angRate.vec() = angularRate;
//		angularRate = (*rotationOffset_ * angRate * rotationInverse).vec();
//	}

	sens.sequenceNumber = packetNumber;
	sens.angularRate = angularRate;

	if (params.useEuler())
	{
		sens.attitude = degToRad(euler);
	}
	else
	{
		sens.attitude = quaternionToEuler(attitude);
	}

	sens.attitude[2] = boundAngleRad(-(sens.attitude[2] - M_PI / 2));

	//Remove gravity from acceleration
	Vector3 gravityInertial(0, 0, 9.81);
	Vector3 gravityBody = Eigen::AngleAxisd(-sens.attitude[0], Vector3::UnitX())
						  * Eigen::AngleAxisd(-sens.attitude[1], Vector3::UnitY()) * gravityInertial;
	sens.acceleration = acceleration + gravityBody;

	//************************
	// GPS Data
	//************************
	double longitude = 0, latitude = 0, altitude = 0;
	double courseAngle = 0;
	if (params.externalGps())
	{
		auto pic = sample->pic_sample;
		if (!pic)
		{
			CPSLOG_ERROR << "PIC sample not available. Cannot read ext GPS data.";
			sens.hasGPSFix = false;
			sens.timestamp = TimePoint();
		}
		else
		{
			sens.hasGPSFix = true;
			const PUBX_POS_fields* gps = &pic->gps_sample.position;
			if (gps->flags != 7)
			{
				gps = &lastGPSSample_;
				if (Clock::now() - gpsSampleTimestamp_ > params.gpsTimeout())
				{
					sens.hasGPSFix = false;
				}
			}
			else
			{
				lastGPSSample_ = *gps;
				gpsSampleTimestamp_ = Clock::now();
			}
			latitude = gps->latitude;
			longitude = gps->longitude;
			altitude = gps->msl_altitude;

			double courseRad = gps->course_gnd * M_PI / 180.; //degrees to radian
			double horSpeed = gps->speed_gnd_kh; // km/h to m/s
			sens.velocity[0] = sin(courseRad) * horSpeed;
			sens.velocity[1] = cos(courseRad) * horSpeed;
			sens.velocity[2] = gps->vert_velocity;
			courseAngle = courseRad;

			sens.courseAngle = courseAngle;

			sens.groundSpeed = sqrt(pow(horSpeed, 2) + pow(gps->vert_velocity, 2));

		}
	}
	else
	{
		imu_sample_t* imu = nullptr;
		if (params.internalImu())
		{
			imu = sample->int_imu_sample;
		}
		else
		{
			imu = sample->imu_sample;
		}

		if (!imu)
		{
			CPSLOG_ERROR << "IMU sample not available. Cannot read GPS data.";
			sens.hasGPSFix = false;
		}
		else
		{
			longitude = imu->imu_lon;
			latitude = imu->imu_lat;
			altitude = imu->imu_alt;

			if (longitude == 0 || latitude == 0 || altitude == 0)
			{
				longitude = lastPosition_.x();
				latitude = lastPosition_.y();
				altitude = lastPosition_.z();
			}
			else
			{
				lastPosition_.x() = longitude;
				lastPosition_.y() = latitude;
				lastPosition_.z() = altitude;
			}

			sens.velocity[0] = imu->imu_vel_x;
			sens.velocity[1] = imu->imu_vel_y;
			sens.velocity[2] = imu->imu_vel_z;

			if (sens.velocity[0] == 0 || sens.velocity[1] == 0 || sens.velocity[2] == 0)
			{
				sens.velocity[0] = lastVelocity_.x();
				sens.velocity[1] = lastVelocity_.y();
				sens.velocity[2] = lastVelocity_.z();
			}
			else
			{
				lastVelocity_.x() = sens.velocity[0];
				lastVelocity_.y() = sens.velocity[1];
				lastVelocity_.z() = sens.velocity[2];
			}

//			courseAngle = atan2(imu->imu_vel_y, imu->imu_vel_x);
			courseAngle = atan2(sens.velocity[1], sens.velocity[0]);
			sens.courseAngle = courseAngle;

			sens.groundSpeed = sens.velocity.norm();

//			sens.hasGPSFix = (sample->imu_sample->valid_flags & 0x80) > 0;
			sens.hasGPSFix = true;
		}
	}

	//Calculate UTM from Wgs84
	int zone;
	char hemi;
	latLongToUTM(latitude, longitude, sens.position[1], sens.position[0], zone, hemi);
	sens.position[2] = altitude;

	//************************
	// Airspeed
	//************************
	if (params.useAirspeed())
	{
		const airs_sample_t* airspeed = sample->airs_sample;
		if (!airspeed)
		{
			CPSLOG_ERROR << "Cannot read airspeed sample. Set airspeed to groundspeed.";
			sens.airSpeed = sens.groundSpeed;
		}
		else
		{
			bool setGroundSpeed = false;
			bool oldAirspeed = false;
			sens.pressure = airspeed->press;
			sens.temperature = airspeed->temp;
			Duration timeDiff;
			if (std::isnan(airspeed->cal_airs) || airspeed->cal_airs == -1)
			{
				oldAirspeed = true;
				airspeed = &lastAirspeedSample_;
				if (Clock::now() - airspeedTimestamp_ > params.airspeedTimeout())
				{
					setGroundSpeed = true;
					sens.airSpeed = sens.groundSpeed; // Set to ground speed if timeout
				}
			}
			else
			{
				lastAirspeedSample_ = *airspeed;
				auto now = Clock::now();
				timeDiff = now - airspeedTimestamp_;
				airspeedTimestamp_ = now;
			}
			if (!setGroundSpeed)
			{
				if (!oldAirspeed)
				{
					airspeedFilter_.update(airspeed->cal_airs,
										   std::chrono::duration_cast<Microseconds>(timeDiff).count() / 1e6);
				}
				sens.airSpeed = airspeedFilter_.getValue();
			}
		}
	}
	else
	{
		sens.airSpeed = sens.groundSpeed;
	}

	//************************
	// PIC Sample Feedback
	//************************
	auto pic = sample->pic_sample;
	if (!pic)
	{
//		CPSLOG_ERROR << "Cannot read PIC sample. Set autopilot active to true.";
		sens.autopilotActive = true;
	}
	else
	{
		sens.autopilotActive = pic->adc_channels[20] > 2000;


		if (auto lsm = get<LinearSensorManager>())
		{
			miscValues_ = lsm->getValues(pic->adc_channels);
		}


		PWMFeedback pwmFeed = {};
		std::copy(pic->pwm_channels, pic->pwm_channels + PWM_CHS, pwmFeed.ch);

		servo.elevator = pic->pwm_channels[3];
		servo.rudder = pic->pwm_channels[4];
		servo.aileron = pic->pwm_channels[5];
	}

	auto slink = sample->slink_sample;
	if (slink)
	{
		power.batteryVoltage = slink->channels->volt;
		power.batteryCurrent = slink->channels->current;
		power.propulsionPower = power.batteryVoltage * power.batteryCurrent;
		servo.throttle = slink->channels->throttle;
		servo.rpm = slink->channels->rpm;
	}

	//************************
	// Calculations
	//************************
	double aoa = sens.attitude[1] - asin(sens.velocity[2] / sens.groundSpeed);
	aoa = aoa > M_PI ? M_PI : aoa < -M_PI ? -M_PI : aoa;
	if (std::isnan(aoa))
		aoa = 0;
	sens.angleOfAttack = aoa;

	Eigen::Matrix3d rotationMatrix;
	Vector3 velocityBody;
	double velocityBodyTotal;
	double velocityBodyLateral;
	double roll = sens.attitude.x();
	double pitch = -sens.attitude.y();
	double yaw = sens.attitude.z();

	rotationMatrix = Eigen::AngleAxisd(-roll, Vector3::UnitX())
					 * Eigen::AngleAxisd(-pitch, Vector3::UnitY())
					 * Eigen::AngleAxisd(-yaw, Vector3::UnitZ());

	velocityBody = rotationMatrix * sens.velocity;
	velocityBodyTotal = velocityBody.norm();
	velocityBodyLateral = velocityBody[1];

	sens.angleOfSideslip = asin(velocityBodyLateral / velocityBodyTotal);

	if (params.courseAsHeading())
	{
		sens.attitude[2] = boundAngleRad(-(courseAngle - M_PI / 2));
	}

	auto aggAPI = get<AggregatableAutopilotAPI>();
	if (!aggAPI)
	{
		CPSLOG_ERROR << "aggAPI missing";
		return -1;
	}

	aggAPI->setSensorData(sens);
	aggAPI->setServoData(servo);
	aggAPI->setPowerData(power);
	return 0;
}

int
ApExtManager::ap_actuate(unsigned long* pwm, unsigned int num_channels)
{
	std::lock_guard<std::mutex> lock(outputMutex_);

	if (num_channels < outputChannels_.size())
	{
		CPSLOG_ERROR << "More output channels than available: " << num_channels << " < "
					 << outputChannels_.size();
		return -1;
	}
	std::copy(outputChannels_.begin(), outputChannels_.end(), pwm);
	OutPWM outputPWM = {};
	memcpy(&outputPWM, pwm, sizeof(OutPWM));

	return 0;
}

void
ApExtManager::onControllerOutput(const ControllerOutput& output)
{
	auto mix = channelMixing_.mapChannels(output, lastAdvancedControl_);
	std::unique_lock<std::mutex> lock(outputMutex_);

	outputChannels_.clear();
	for (const auto& channel : mix)
	{
		outputChannels_.push_back(round(channel));
	}

	lock.unlock();
}

void
ApExtManager::onAdvancedControl(const AdvancedControl& control)
{
	std::lock_guard<std::mutex> lock(advancedControlMutex_);
	lastAdvancedControl_ = control;
}

std::map<std::string, FloatingType>
ApExtManager::getMiscValues() const
{
	return miscValues_;
}
