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
 * @file IDCFactory.h
 * @brief Defines the IDCFactory
 * @date Jul 31, 2017
 * @author Mirco Theile, mirco.theile@tum.de
 */

#ifndef UAVAP_CORE_IDC_NETWORKFACTORY_H_
#define UAVAP_CORE_IDC_NETWORKFACTORY_H_
#include "uavAP/Core/IDC/NetworkLayer/INetworkLayer.h"
#include "uavAP/Core/IDC/NetworkLayer/Serial/SerialNetworkLayer.h"
#include "uavAP/Core/Framework/Factory.h"

/**
 * @brief Factory for IInterDeviceComm objects. Default creates SerialIDC
 */
class NetworkFactory: public Factory<INetworkLayer>
{
public:
	NetworkFactory()
	{
		addCreator<SerialNetworkLayer>();
	}

	static constexpr TypeId typeId = "network";

};

#endif /* UAVAP_CORE_IDC_NETWORKFACTORY_H_ */
