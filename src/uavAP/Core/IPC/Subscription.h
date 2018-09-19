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
 * Subscription.h
 *
 *  Created on: Aug 3, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IPC_SUBSCRIPTION_H_
#define UAVAP_CORE_IPC_SUBSCRIPTION_H_
#include <boost/signals2/connection.hpp>
#include "uavAP/Core/IPC/detail/ISubscriptionImpl.h"
#include <memory>

class Subscription
{
public:

	Subscription();

	Subscription(std::shared_ptr<ISubscriptionImpl> impl, const boost::signals2::connection& con);

	void
	cancel();

	bool
	connected();

private:

	std::weak_ptr<ISubscriptionImpl> subscriptionImpl_;

	boost::signals2::connection connection_;

};

#endif /* UAVAP_CORE_IPC_SUBSCRIPTION_H_ */
