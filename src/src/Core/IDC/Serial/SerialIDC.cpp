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
 * SerialIDC.cpp
 *
 *  Created on: Jul 31, 2017
 *      Author: mircot
 */
#include "uavAP/Core/IDC/IIDCParams.h"
#include "uavAP/Core/IDC/Serial/SerialIDC.h"
#include "uavAP/Core/IDC/Serial/SerialIDCParams.h"
#include "uavAP/Core/IDC/Serial/SerialSenderImpl.h"
#include "uavAP/Core/Logging/APLogger.h"

Sender
SerialIDC::createSender(const IIDCParams& params)
{
	try
	{
		auto serialParams = dynamic_cast<const SerialIDCParams&>(params);
		auto impl = std::make_shared<SerialSenderImpl>(serialParams);
		sender_.push_back(impl);
		return Sender(impl);
	} catch (std::bad_cast&)
	{
		APLOG_ERROR << "Provided params are not serial params.";
		return Sender();
	} catch (boost::system::system_error& err)
	{
		APLOG_ERROR << "System error with "
				<< dynamic_cast<const SerialIDCParams&>(params).serialPort << ": " << err.what();
		return Sender();
	}
}

void
SerialIDC::subscribeOnPacket(const IIDCParams& params, const std::function<void
(const Packet&)>& handle)
{
	try
	{
		auto serialParams = dynamic_cast<const SerialIDCParams&>(params);
		auto receiver = std::make_shared<SerialReceiver>(serialParams, handle);
		receiver_.push_back(receiver);
		auto sched = scheduler_.get();
		if (!sched)
		{
			APLOG_ERROR << "Cannot subscribe. Scheduler missing.";
			return;
		}
		sched->schedule(std::bind(&SerialReceiver::startReceive, receiver), Milliseconds(0));
	} catch (std::bad_cast&)
	{
		APLOG_ERROR << "Provided params are not serial params.";
	} catch (boost::system::system_error& err)
	{
		APLOG_ERROR << "System error with "
				<< dynamic_cast<const SerialIDCParams&>(params).serialPort << ": " << err.what();
	}
}

std::shared_ptr<IInterDeviceComm>
SerialIDC::create(const boost::property_tree::ptree& config)
{
	return std::make_shared<SerialIDC>();
}

void
SerialIDC::notifyAggregationOnUpdate(Aggregator& agg)
{
	scheduler_.setFromAggregationIfNotSet(agg);
}
