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
 * @file SerializeCustom.h
 * @brief Defines the SerializeCustom struct
 * @date Sep 17, 2017
 * @author Mirco Theile, mirco.theile@tum.de
 */

#ifndef UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_SERIALIZECUSTOM_H_
#define UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_SERIALIZECUSTOM_H_

/**
 * @brief 	defines an empty struct that disables auto serialization for the object inheriting
 * 			from it. Sometimes necessary if the compiler wants to serialize the object as e.g.
 * 			a POD type, but the serialization is more complex.
 */
struct SerializeCustom
{
};

#endif /* UAVAP_CORE_DATAPRESENTATION_APDATAPRESENTATION_SERIALIZECUSTOM_H_ */
