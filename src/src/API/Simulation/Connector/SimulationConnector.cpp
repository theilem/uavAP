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
 * SimulationConnector.cpp
 *
 *  Created on: Jul 28, 2017
 *      Author: mircot
 */
#include <boost/asio/read_until.hpp>
#include "uavAP/API/ChannelMixing.h"
#include "uavAP/API/Simulation/Connector/Codec.h"
#include "uavAP/API/Simulation/Connector/SimulationConnector.h"
#include "uavAP/Core/DataPresentation/IDataPresentation.h"
#include "uavAP/Core/IDC/Serial/SerialIDC.h"
#include "uavAP/Core/IDC/Serial/SerialIDCParams.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/SensorData.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"

SimulationConnector::SimulationConnector() :
    logging_(false)
{
}

SimulationConnector::~SimulationConnector()
{
    if (fileStream_.is_open())
        fileStream_.close();
}

bool
SimulationConnector::run(RunStage stage)
{
    switch (stage)
    {
    case RunStage::INIT:
    {
        if (!idc_.isSet())
        {
            APLOG_ERROR << "SimulationConnector: SerialIDC missing.";
            return true;
        }
        if (!ipc_.isSet())
        {
            APLOG_ERROR << "SimulationConnector: IPC missing.";
            return true;
        }
        if (!scheduler_.isSet())
        {
            APLOG_ERROR << "SimulationConnector: Scheduler missing.";
            return true;
        }
        if (!channelMixing_.isSet())
        {
            APLOG_ERROR << "SimulationConnector: ChannelMixing missing.";
            return true;
        }
        auto ipc = ipc_.get();
        sensorPublisher_ = ipc->publishOnSharedMemory<SensorData>("sensor_data");
        break;
    }
    case RunStage::NORMAL:
    {
        tryConnectActuation();
        tryConnectCommunication();
        auto idc = idc_.get();
        SerialIDCParams params(serialPort_, 115200, "\0");
        idc->subscribeOnPacket(params,
                               std::bind(&SimulationConnector::sense, this, std::placeholders::_1));
        SerialIDCParams actuationParams(serialPort_, 115200, "\n");
        actuationSender_ = idc->createSender(actuationParams);
        break;
    }
    case RunStage::FINAL:
    {
        break;
    }
    default:
        break;
    }
    return false;
}

void
SimulationConnector::notifyAggregationOnUpdate(Aggregator& agg)
{
    ipc_.setFromAggregationIfNotSet(agg);
    scheduler_.setFromAggregationIfNotSet(agg);
    timeProvider_.setFromAggregationIfNotSet(agg);
    idc_.setFromAggregationIfNotSet(agg);
    channelMixing_.setFromAggregationIfNotSet(agg);
    dataPresentation_.setFromAggregationIfNotSet(agg);
}

void
SimulationConnector::sense(const Packet& packet)
{
    std::string decoded = decode(packet.getBuffer());

    SensorData sd;
    Vector3 position;
    int parsed = sscanf(decoded.c_str(),
                        "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf", &sd.acceleration[0],
                        &sd.acceleration[1], &sd.acceleration[2], &sd.angularRate[0], &sd.angularRate[1],
                        &sd.angularRate[2], &sd.attitude[0], &sd.attitude[1], &sd.attitude[2], &position[1],
                        &position[0], &position[2], &sd.velocity[0], &sd.velocity[1], &sd.velocity[2]);
    if (parsed != 15)
    {
        APLOG_TRACE << "Data received not complete. ignore.";
        return;
    }
    //Convert position from END to ENU
    position[2] *= -1;
    sd.position = position;

    sd.velocityGround = sd.velocity[0]; // TODO correct groundspeed airspeed difference
    sd.velocityAir = sd.velocity[0];

    Eigen::Matrix3d m;
    m = Eigen::AngleAxisd(sd.attitude.z(), Vector3::UnitZ());

    sd.velocity = m * sd.velocity;



    //Convert to ENU
    double velNorth = sd.velocity[0];
    sd.velocity[0] = sd.velocity[1];
    sd.velocity[1] = velNorth;
    sd.velocity[2] *= -1;

    sd.hasGPSFix = true;
    sd.autopilotActive = true;

    sd.timestamp = boost::get_system_time();
    sensorPublisher_.publish(sd);

    if (logging_)
    {
        logData(sd);
    }
}

void
SimulationConnector::actuate(const ControllerOutput& out)
{
    std::stringstream ss;

    auto chMix = channelMixing_.get();
    if (!chMix)
    {
        APLOG_ERROR << "Channel mixing missing.";
        return;
    }

    auto output = chMix->mixChannels(out);

    for (auto it : output)
    {
        ss << it << ",";
    }

    ss << "1\r";

    actuationSender_.sendPacket(Packet(ss.str()));
}

bool
SimulationConnector::configure(const boost::property_tree::ptree& config)
{
    PropertyMapper pm(config);

    pm.add("serial_port", serialPort_, true);
    pm.add("log", logging_, false);

    if (logging_)
    {
        if (!pm.add("log_path", loggingPath_, true))
            logging_ = false;
    }

    return pm.map();
}

void
SimulationConnector::tryConnectActuation()
{
    auto ipc = ipc_.get();
    actuationSubscription_ = ipc->subscribeOnSharedMemory<ControllerOutput>("actuation",
                             boost::bind(&SimulationConnector::actuate, this, _1));
    if (!actuationSubscription_.connected())
    {
        APLOG_DEBUG << "Was not able to subscribe to actuation. Try again in 1sec.";
        auto sched = scheduler_.get();
        sched->schedule(std::bind(&SimulationConnector::tryConnectActuation, this), Seconds(1));
    }
}

void
SimulationConnector::tryConnectCommunication()
{
    auto ipc = ipc_.get();
    communicationSubscription_ = ipc->subscribeOnPacket("data_com_api",
                                 boost::bind(&SimulationConnector::receivePackets, this, _1));
    if (!communicationSubscription_.connected())
    {
        auto sched = scheduler_.get();
        sched->schedule(std::bind(&SimulationConnector::tryConnectCommunication, this), Seconds(1));
    }
}

std::shared_ptr<SimulationConnector>
SimulationConnector::create(const boost::property_tree::ptree& config)
{
    auto sim = std::make_shared<SimulationConnector>();
    sim->configure(config);
    return sim;
}

void
SimulationConnector::receivePackets(const Packet& packet)
{
    auto dp = dataPresentation_.get();
    if (!dp)
        return;

    Content content = Content::INVALID;
    auto any = dp->deserialize(packet, content);
    if (content == Content::REQUEST_DATA)
    {
        auto req = boost::any_cast<DataRequest>(any);
        if (req == DataRequest::START_LOGGING)
            startLogging();
        else if (req == DataRequest::STOP_LOGGING)
            stopLogging();
    }
    //
    //	auto sched = scheduler_.get();
    //	if (!sched)
    //		return;
    //	actuationResetEvent_.cancel();
    //	actuationResetEvent_ = sched->schedule(std::bind(&SimulationConnector::actuationDisconnect, this), Seconds(1));
}

void
SimulationConnector::startLogging()
{
    if (!logging_)
        return;
    if (fileStream_.is_open())
        return;

    fileStream_.open(loggingPath_, std::ios_base::out);
}

void
SimulationConnector::stopLogging()
{
    if (!fileStream_.is_open())
        return;

    fileStream_.close();
}

void
SimulationConnector::actuationDisconnect()
{
    APLOG_DEBUG << "Actuation Disconnected. Try reconnecting.";

    actuationSubscription_.cancel();

    tryConnectActuation();
}

void
SimulationConnector::logData(const SensorData& data)
{
    if (!fileStream_.is_open())
        return;

    fileStream_ << std::setprecision(8) << data.velocity.x() << ";";
    fileStream_ << std::setprecision(8) << data.velocity.y() << ";";
    fileStream_ << std::setprecision(8) << data.velocity.z() << ";";
    fileStream_ << std::setprecision(8) << data.acceleration.x() << ";";
    fileStream_ << std::setprecision(8) << data.acceleration.y() << ";";
    fileStream_ << std::setprecision(8) << data.acceleration.z() << ";";
    fileStream_ << std::setprecision(8) << data.attitude.x() * 180. / M_PI << ";";
    fileStream_ << std::setprecision(8) << data.attitude.y() * 180. / M_PI << ";";
    fileStream_ << std::setprecision(8) << data.attitude.z() * 180. / M_PI << ";";
    fileStream_ << std::setprecision(8) << data.angularRate.x() * 180. / M_PI << ";";
    fileStream_ << std::setprecision(8) << data.angularRate.y() * 180. / M_PI << ";";
    fileStream_ << std::setprecision(8) << data.angularRate.z() * 180. / M_PI << ";";
    fileStream_ << std::setprecision(8) << data.velocityAir << ";";
    fileStream_ << std::setprecision(8) << data.velocityGround << ";";
    fileStream_ << "\n";
}

