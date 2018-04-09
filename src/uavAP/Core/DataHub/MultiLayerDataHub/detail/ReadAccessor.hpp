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
 * ReadAccessor.hpp
 *
 *  Created on: Feb 26, 2018
 *      Author: uav
 */

#ifndef UAVAP_CORE_DATAHUB_MULTILAYERDATAHUB_DETAIL_READACCESSOR_HPP_
#define UAVAP_CORE_DATAHUB_MULTILAYERDATAHUB_DETAIL_READACCESSOR_HPP_


#include <atomic>

class ReadAccessor
{
public:

	template <typename Type>
	Type
	getValue();

	template <typename Type>
	Type
	getValue(const std::string& path);

private:

	std::unordered_map<std::string, ReadAccessor> children_;

	std::atomic_int& counter_;

	std::type_info& dataType_;
	void* data_;




};



#endif /* UAVAP_CORE_DATAHUB_MULTILAYERDATAHUB_DETAIL_READACCESSOR_HPP_ */
