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
 * ApExtManager.cpp
 *
 *  Created on: Aug 31, 2017
 *      Author: mircot
 */
#include "uavAP/API/ap_ext/ApExtManager.h"
#include "uavAP/API/ap_ext/latLongToUTM.h"
#include "uavAP/API/ChannelMixing.h"
#include "uavAP/Core/DataPresentation/APDataPresentation/APDataPresentation.h"
#include "uavAP/Core/Runner/SimpleRunner.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/Scheduler/MultiThreadingScheduler.h"
#include "uavAP/Core/TimeProvider/SystemTimeProvider.h"
#include <cmath>

ApExtManager::ApExtManager() :
    ipc_(std::make_shared<IPC>()), timeProvider_(std::make_shared<SystemTimeProvider>()), scheduler_(
        std::make_shared<MultiThreadingScheduler>()), dataPresentation_(
            std::make_shared<APDataPresentation<Content, Target>>()), channelMixing_(
                std::make_shared<ChannelMixing>())
{

    agg_.add(ipc_);
    agg_.add(timeProvider_);
    agg_.add(scheduler_);
    agg_.add(channelMixing_);
    agg_.add(dataPresentation_);

    SimpleRunner runner(agg_);
    runner.runAllStages();

    sensorDataPublisher_ = ipc_->publishOnSharedMemory<SensorData>("sensor_data");
    outputPublisher_ = ipc_->publishOnSharedMemory<OutPWM>("output_pwm");
}

bool
ApExtManager::configure(const boost::property_tree::ptree& config)
{
    bool success = channelMixing_->configure(config) && servoMapping_.configure(config);
    tryConnectControllerOut();

    return success;
}

int
ApExtManager::ap_sense(const data_sample_t* sample)
{
    SensorData sens;
    /* Acceleration */
    sens.acceleration[0] = sample->imu_sample->imu_accel_x;
    sens.acceleration[1] = sample->imu_sample->imu_accel_y;
    sens.acceleration[2] = sample->imu_sample->imu_accel_z;

    /* Rotation rate */
    sens.angularRate[0] = sample->imu_sample->imu_rot_x;
    sens.angularRate[1] = sample->imu_sample->imu_rot_y;
    sens.angularRate[2] = sample->imu_sample->imu_rot_z;

    /* Euler angles */
    double pi_rad = M_PI / 180;

    double rad_roll = sample->imu_sample->imu_euler_roll * pi_rad;
    double rad_pitch = sample->imu_sample->imu_euler_pitch * pi_rad;
    double rad_yaw = sample->imu_sample->imu_euler_yaw * pi_rad;

    sens.attitude[0] = rad_roll;
    sens.attitude[1] = rad_pitch;
    sens.attitude[2] = rad_yaw;

    //Remove gravity from acceleration
    Vector3 gravityInertial(0, 0, 9.81);
    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(-rad_roll, Vector3::UnitX())
        * Eigen::AngleAxisd(-rad_pitch, Vector3::UnitY());
    Vector3 gravityBody = m * gravityInertial;
    sens.acceleration = sens.acceleration + gravityBody;

    /*Compute ENU Position */
    int zone;
    char hemi;
    latLongToUTM(sample->imu_sample->imu_lat, sample->imu_sample->imu_lon, sens.position[1],
                 sens.position[0], zone, hemi);
    sens.position[2] = sample->imu_sample->imu_alt;

    /* Velocity ENU*/
    sens.velocity[0] = sample->imu_sample->imu_vel_x;
    sens.velocity[1] = sample->imu_sample->imu_vel_y;
    sens.velocity[2] = sample->imu_sample->imu_vel_z;

    /* Groundspeed */
    double cosTheta = cos(rad_pitch);
    double cosPsi = cos(rad_yaw);
    double sinTheta = sin(rad_pitch);
    double sinPsi = sin(rad_yaw);
    sens.velocityGround = sample->imu_sample->imu_vel_y * cosTheta * cosPsi
                          + sample->imu_sample->imu_vel_x * cosTheta * sinPsi
                          + sample->imu_sample->imu_vel_z * sinTheta;

    /* Airspeed */ //TODO Conversion
//	sens.velocityAir = sample->pic_sample->adc_channels[AIRSPEED_ADC_CH];
    sens.velocityAir = sens.velocityGround;

    /* Angle of Attack */
    double aoa = rad_pitch - asin(sens.velocity[2] / sens.velocityGround);
    aoa = aoa > M_PI ? M_PI : aoa < -M_PI ? -M_PI : aoa;
    if (std::isnan(aoa))
        aoa = 0;
    sens.angleOfAttack = aoa;

    /* Timestamp */
    sens.hasGPSFix = (sample->imu_sample->valid_flags & 0x80) > 0;

    if (sens.hasGPSFix)
    {
        try
        {
            Date date(sample->imu_sample->imu_time_year, sample->imu_sample->imu_time_month,
                      sample->imu_sample->imu_time_day);

            Duration duration = Hours(sample->imu_sample->imu_time_hour)
                                + Minutes(sample->imu_sample->imu_time_minute)
                                + Seconds(sample->imu_sample->imu_time_second)
                                + Microseconds(sample->imu_sample->imu_time_nano / 1000);
            sens.timestamp = TimePoint(date, duration);
        }
        catch (std::out_of_range& err)
        {
            APLOG_ERROR << "Time is not valid. " << err.what();
            sens.timestamp = boost::posix_time::not_a_date_time;
        }
    }
    else
    {
        sens.timestamp = boost::posix_time::not_a_date_time;
    }

    sens.autopilotActive = sample->pic_sample->adc_channels[20] > 2000;

    sens.sequenceNr = static_cast<uint32_t>(sample->pic_sample->pwm_channels[21]);

    sensorDataPublisher_.publish(sens);
    return 0;
}

int
ApExtManager::ap_actuate(unsigned long* pwm, unsigned int num_channels)
{
    std::lock_guard<std::mutex> lock(outputMutex_);

    if (num_channels < outputChannels_.size())
    {
        APLOG_ERROR << "More output channels than available: " << num_channels << " < "
                    << outputChannels_.size();
        return -1;
    }
    std::copy(outputChannels_.begin(), outputChannels_.end(), pwm);
    OutPWM outputPWM;
    memcpy(&outputPWM, pwm, sizeof(OutPWM));

    outputPublisher_.publish(outputPWM);
    return 0;
}

void
ApExtManager::onControllerOutput(const ControllerOutput& output)
{
    auto mix = channelMixing_->mixChannels(output);
    std::unique_lock<std::mutex> lock(outputMutex_);
    outputChannels_ = servoMapping_.map(mix);

    outputChannels_.push_back(static_cast<unsigned long>(output.sequenceNr));
    lock.unlock();

    if (notifyActuation_)
        notifyActuation_();
}

void
ApExtManager::onServoOutput(const OutPWM& output)
{
    std::lock_guard<std::mutex> lock(outputMutex_);
    outputChannels_ = std::vector<unsigned long>(output.ch,
                      output.ch + sizeof(output.ch) / sizeof(output.ch[0]));
}

void
ApExtManager::tryConnectControllerOut()
{
    APLOG_DEBUG << "Try connect to controller out.";
    controllerOutSubscription_ = ipc_->subscribeOnSharedMemory<ControllerOutput>("actuation",
                                 boost::bind(&ApExtManager::onControllerOutput, this, _1));
    servoOutSubscription_ = ipc_->subscribeOnSharedMemory<OutPWM>("servo_out",
                            boost::bind(&ApExtManager::onServoOutput, this, _1));
    if (!controllerOutSubscription_.connected())
    {
        APLOG_DEBUG << "Was not able to subscribe to actuation. Try again in 1sec.";
        scheduler_->schedule(std::bind(&ApExtManager::tryConnectControllerOut, this), Seconds(1));
    }
    if (servoOutSubscription_.connected())
        APLOG_WARN << "Servo Override activated.";
}

void
ApExtManager::notifyOnActuation(const NotifyActuation& slot)
{
    notifyActuation_ = slot;
}
