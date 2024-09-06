/*
 * AutopilotInterface.cpp
 *
 *  Created on: Nov 25, 2017
 *      Author: mircot
 */

#include <uavAP/API/ap_ext/ap_ext.h>
#include <uavAP/API/ap_ext/ApExtManager.h>
#include <uavAP/FlightControl/Controller/ControllerOutput.h>
#include <cpsCore/Utilities/IDC/IDC.h>
#include <cpsCore/Utilities/IDC/NetworkLayer/Serial/SerialNetworkLayerParams.h>

#include "UTMToLatLong.h"
#include "EmulationAPInterface.h"

EmulationAPInterface::EmulationAPInterface() :
		setup_(false), numChannels_(7)
{
}

EmulationAPInterface::~EmulationAPInterface()
{
	ap_ext_teardown();
}

std::shared_ptr<EmulationAPInterface>
EmulationAPInterface::create(const Configuration& config)
{
	auto emulation = std::make_shared<EmulationAPInterface>();
	if (!emulation->configure(config))
		CPSLOG_ERROR << "Configuration of EmulationAPInterface not successfull.";
	return emulation;
}

bool
EmulationAPInterface::configure(const Configuration& config)
{
	PropertyMapper pm(config);
	pm.add("serial_port", serialPort_, true);
	return channelMixing_.configure(config) && servoMapping_.configure(config) && pm.map();
}

bool
EmulationAPInterface::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!checkIsSet<IDC, DataPresentation, IScheduler>())
		{
			CPSLOG_ERROR << "EmulationAPInterface: missing dependencies.";
			return true;
		}
		break;
	}
	case RunStage::NORMAL:
	{
		auto idc = get<IDC>();
		auto sched = get<IScheduler>();

		SerialNetworkParams params(serialPort_, 115200, "*-*");
		idc->subscribeOnPacket(params,
				std::bind(&EmulationAPInterface::onPacket, this, std::placeholders::_1));
		actuationSender_ = idc->createSender(params);

		dataSample_.imu_sample = new imu_sample_t;
		dataSample_.pic_sample = new pic_sample_t;

		dataSample_.pic_sample->adc_channels[20] = 3000; //Autopilot always active TODO: Maybe not?

		if (ap_ext_setup() != 0)
			return true;
//		sched->schedule(std::bind(&EmulationAPInterface::sendActuation, this), Milliseconds(0), Milliseconds(2));
		auto apMan = getApExtManager();
		apMan->notifyOnActuation(std::bind(&EmulationAPInterface::sendActuation, this));
		setup_ = true;
		break;
	}
	case RunStage::FINAL:
		break;
	default:
		break;
	}
	return false;
}

void
EmulationAPInterface::onSensorData(const SensorData& sensorData)
{
	if (!setup_)
	{
		CPSLOG_WARN << "ApExt not setup. Cannot send sensor data to ap.";
		return;
	}

	imu_sample_t* imu = dataSample_.imu_sample;

	/* Rotation rate */
	imu->imu_rot_x = sensorData.angularRate.x();
	imu->imu_rot_y = sensorData.angularRate.y();
	imu->imu_rot_z = sensorData.angularRate.z();

	double roll = sensorData.attitude.x();
	double pitch = sensorData.attitude.y();
	double yaw = sensorData.attitude.z();

	//Add gravity to acceleration
	Vector3 gravityInertial(0, 0, 9.81);
	Matrix3 m;
	m = AngleAxis(-roll, Vector3::UnitX()) * AngleAxis(-pitch, Vector3::UnitY());
	Vector3 gravityBody = m * gravityInertial;

	imu->imu_accel_x = sensorData.acceleration.x() - gravityBody.x();
	imu->imu_accel_y = sensorData.acceleration.y() - gravityBody.y();
	imu->imu_accel_z = sensorData.acceleration.z() - gravityBody.z();

	double toDegree = 180. / M_PI;
	roll *= toDegree;
	pitch *= toDegree;
	yaw *= toDegree;

	imu->imu_euler_roll = roll;
	imu->imu_euler_pitch = pitch;
	imu->imu_euler_yaw = yaw;

	/* North, East Down coordinates - computed aside */
	/* Compute north, east, down coordinates */

	UTMtoLL(22, sensorData.position.y(), sensorData.position.x(), 0, imu->imu_lat, imu->imu_lon);
	imu->imu_alt = sensorData.position.z();

	/* Velocity */
	imu->imu_vel_x = sensorData.velocity.x();
	imu->imu_vel_y = sensorData.velocity.y();
	imu->imu_vel_z = sensorData.velocity.z();

	//Valid flag for GPS fix
	imu->valid_flags = 0x80;

	/* Timestamp */

	if (!sensorData.timestamp.is_not_a_date_time())
	{
		auto time = boost::posix_time::to_tm(sensorData.timestamp);

		imu->imu_time_year = time.tm_year + 1900; //Year offset of ROS
		imu->imu_time_month = time.tm_mon + 1;
		imu->imu_time_day = time.tm_mday;
		imu->imu_time_hour = time.tm_hour;
		imu->imu_time_minute = time.tm_min;
		imu->imu_time_second = time.tm_sec;
		imu->imu_time_nano = sensorData.timestamp.time_of_day().total_nanoseconds();
	}
	else
	{
		CPSLOG_WARN << "Timestamp not set.";
	}

	dataSample_.pic_sample->pwm_channels[21] = static_cast<unsigned long>(sensorData.sequenceNr);

	int sensResult = ap_ext_sense(&dataSample_);

	if (sensResult != 0)
	{
		CPSLOG_WARN << "Something went wrong sending the data sample.";
	}
}

void
EmulationAPInterface::onPacket(const Packet& packet)
{
	CPSLOG_DEBUG << "Received packet.";
	auto dp = dataPresentation_.get();
	if (!dp)
	{
		CPSLOG_ERROR << "Data presentation missing. Cannot deserialize packet.";
		return;
	}

	Content content = Content::INVALID;
	auto any = dp->deserialize(packet, content);

	if (content == Content::SENSOR_DATA)
	{
		onSensorData(boost::any_cast<SensorData>(any));
	}
	else if (content == Content::SENSOR_DATA_LIGHT)
	{
		onSensorData(fromSensorDataLight(boost::any_cast<SensorDataLight>(any)));
	}
	else
	{
		CPSLOG_ERROR
				<< "Invalid packet received. Only sensor data and sensorDataLight allowed. Content: "
				<< static_cast<int>(content);
		return;
	}

}

void
EmulationAPInterface::sendActuation()
{
	unsigned long pwmSeq[numChannels_ + 1];

	int actResult = ap_ext_actuate(pwmSeq, numChannels_ + 1);

	uint32_t seq = static_cast<uint32_t>(pwmSeq[numChannels_]);
//	int actResult = ap_ext_actuate(pwm, numChannels_);

	//Do only write when there was no update in sensordata
	if (lastSequenceNr_ == seq)
		return;

	unsigned long pwm[numChannels_];
	std::copy(pwmSeq, pwmSeq + numChannels_, pwm);

	lastSequenceNr_ = seq;

	if (actResult != 0)
	{
		CPSLOG_WARN << "Something went wrong getting actuation data.";
		return;
	}

	auto act = reverseChannelMixing(pwm);

	act.sequenceNr = seq;

	auto dp = dataPresentation_.get();
	if (!dp)
	{
		CPSLOG_ERROR << "Data presentation missing. Cannot send Actuation.";
		return;
	}
	auto actLight = fromControllerOutput(act);
	auto packet = dp->serialize(actLight, Content::CONTROLLER_OUTPUT_LIGHT);
	actuationSender_.sendPacket(packet);
}

ControllerOutput
EmulationAPInterface::reverseChannelMixing(unsigned long * pwm)
{
	std::vector<unsigned long> pwmVec(pwm, pwm + numChannels_);

	auto channels = servoMapping_.unmap(pwmVec);

	return channelMixing_.unmixChannels(channels);
}
