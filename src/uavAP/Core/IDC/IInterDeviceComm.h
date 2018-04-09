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
 * @file IInterDeviceComm.h
 * @brief Defines the Interface class IInterDeviceComm
 * @date Jul 31, 2017
 * @author Mirco Theile, mircot@illinois.edu
 */

#ifndef UAVAP_CORE_IDC_IINTERDEVICECOMM_H_
#define UAVAP_CORE_IDC_IINTERDEVICECOMM_H_
#include "uavAP/Core/IDC/Sender.h"
#include "uavAP/Core/DataPresentation/Packet.h"

struct IIDCParams;

/**
 * @brief Interface class for the inter device communication (IDC).
 *
 * For IDC an InterDeviceComm should have the ability to create a sender to send packets and subscribe on
 * incoming packets by calling a handle.
 */
class IInterDeviceComm
{
public:

	virtual
	~IInterDeviceComm() = default;

	/**
	 * @brief 	Create a Sender object that allows to send packets though IDC using the IIDCParams
	 * 			defined in params
	 * @param params Parameters defining the IDC settings
	 * @return A Sender object
	 */
	virtual Sender
	createSender(const IIDCParams& params) = 0;

	/**
	 * @brief Subscribe on packets coming through IDC. Uses the params to set up the reception.
	 * @param params Parameters defining the IDC settings
	 * @param handle Handle to be called with the received packet
	 */
	virtual void
	subscribeOnPacket(const IIDCParams& params, const std::function<void(const Packet&)>& handle) = 0;
};



#endif /* UAVAP_CORE_IDC_IINTERDEVICECOMM_H_ */
