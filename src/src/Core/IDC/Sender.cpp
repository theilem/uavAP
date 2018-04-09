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
 * Sender.cpp
 *
 *  Created on: Jul 31, 2017
 *      Author: mircot
 */
#include "uavAP/Core/DataPresentation/Packet.h"
#include "uavAP/Core/IDC/Sender.h"
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/IDC/ISenderImpl.h"

Sender::Sender(std::shared_ptr<ISenderImpl> impl) :
		senderImpl_(impl)
{
}

bool
Sender::sendPacket(const Packet& packet)
{
	if (auto sender = senderImpl_.lock())
	{
		return sender->sendPacket(packet);
	}
	APLOG_ERROR << "Sender incomplete. Missing SenderImpl.";
	return false;
}

bool
Sender::isConnected()
{
	return senderImpl_.lock() != nullptr;
}
