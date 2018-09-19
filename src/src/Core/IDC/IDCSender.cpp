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
 * IDCSender.cpp
 *
 *  Created on: Jul 28, 2018
 *      Author: mircot
 */
#include <uavAP/Core/DataPresentation/Packet.h>
#include <uavAP/Core/IDC/IDC.h>
#include <uavAP/Core/IDC/IDCSender.h>
#include <uavAP/Core/Logging/APLogger.h>




IDCSender::IDCSender(std::weak_ptr<IDC> idc, const std::string& id):
	idc_(idc), id_(id)
{
}

bool
IDCSender::sendPacket(const Packet& packet, bool ack)
{
	auto idc = idc_.lock();
	if (!idc)
	{
		APLOG_WARN << "IDC not available";
		return false;
	}

	return idc->sendPacket(id_, packet);
}
