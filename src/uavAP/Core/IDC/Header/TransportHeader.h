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
 * TransportHeader.h
 *
 *  Created on: Jul 27, 2018
 *      Author: mircot
 */

#ifndef UAVAP_CORE_IDC_TRANSPORTHEADER_H_
#define UAVAP_CORE_IDC_TRANSPORTHEADER_H_
#include <cstdint>

struct TransportHeader
{
	uint16_t sequenceNr;
	uint8_t flags; //!< ack_requested, ack_present, segmented, 0, 0, 0, 0, 0
	uint16_t ackNr;
	uint16_t packetId;
	uint8_t segmentationNr;
	uint8_t segmentationTotal;
};

namespace dp
{
template<class Archive, typename Type>
void
serialize(Archive& ar, TransportHeader& t)
{
	ar & t.sequenceNr;
	ar & t.flags;
	if (t.flags & 0x02)
	{
		ar & t.ackNr;
	}
	if (t.flags & 0x04)
	{
		ar & t.packetId;
		ar & t.segmentationNr;
		ar & t.segmentationTotal;
	}
}
}

#endif /* UAVAP_CORE_IDC_TRANSPORTHEADER_H_ */
