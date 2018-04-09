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
/**
 * @file SerialIDC.h
 * @brief Defines SerialIDC
 * @date Jun 12, 2017
 * @author Mirco Theile, mircot@illinois.edu
 */
#ifndef GROUNDCOMMUNICATION_SERIALCONNECTION_H_
#define GROUNDCOMMUNICATION_SERIALCONNECTION_H_

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/signals2/signal.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/system/error_code.hpp>
#include "uavAP/Core/IDC/IInterDeviceComm.h"
#include "uavAP/Core/IDC/Serial/SerialReceiver.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include "uavAP/Core/Object/ObjectHandle.h"
#include "uavAP/Core/Scheduler/IScheduler.h"

class SerialSenderImpl;

/**
 * @brief IDC using serial to communicate among devices
 */
class SerialIDC: public IInterDeviceComm, public IAggregatableObject
{

public:

	/**
	 * @brief Create the SerialIDC using a config tree
	 * @param config Configuration tree
	 * @return IInterDeviceComm ptr of a SerialIDC
	 */
	static std::shared_ptr<IInterDeviceComm>
	create(const boost::property_tree::ptree& config);

	/**
	 * @brief Create a Sender object using params. The params have to be SerialIDCParams
	 * @param params SerialIDCParams defining the serial port etc.
	 * @return Sender object
	 */
	Sender
	createSender(const IIDCParams& params) override;

	/**
	 * @brief Subscribe on incoming packets. Uses params to create serial port. Needs scheduler to
	 * 		dedicate a thread for waiting for reception.
	 * @param params SerialIDCParams defining the serial port etc.
	 * @param handle handler to be called with incoming packet
	 */
	void
	subscribeOnPacket(const IIDCParams& params, const std::function<void
	(const Packet&)>& handle) override;

	/**
	 * @brief Set IScheduler from aggregation
	 * @param agg Aggregator containng the aggregation
	 */
	void
	notifyAggregationOnUpdate(Aggregator& agg);

private:

	std::vector<std::shared_ptr<SerialSenderImpl> > sender_; //!< Vector containing the SerialSenderImpl

	std::vector<std::shared_ptr<SerialReceiver> > receiver_; //!< Vector containing the SerialReceiver

	ObjectHandle<IScheduler> scheduler_; //!< scheduler from aggregation

};

#endif /* GROUNDCOMMUNICATION_SERIALCONNECTION_H_ */
