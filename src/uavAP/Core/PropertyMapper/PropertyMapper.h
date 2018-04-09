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
 * PropertyMapper.h
 *
 *  Created on: Jun 17, 2017
 *      Author: mircot
 */

#ifndef UTIL_PROPERTYMAPPER_H_
#define UTIL_PROPERTYMAPPER_H_

#include <boost/property_tree/ptree.hpp>
#include <google/protobuf/message.h>
#include "uavAP/Core/Logging/APLogger.h"
#include "uavAP/Core/Time.h"

class PropertyMapper
{
public:
	PropertyMapper(const boost::property_tree::ptree& p);

	template<typename T>
	bool
	add(std::string key, T& val, bool mandatory);

	template<typename T>
	bool
	addVector(std::string key, std::vector<T>& val, bool mandatory);

	bool
	addVector(std::string key, std::vector<boost::property_tree::ptree>& val, bool mandatory);

	bool
	add(std::string key, Duration& val, bool mandatory);

	bool
	add(std::string key, boost::property_tree::ptree& val, bool mandatory);

	bool
	add(std::string key, google::protobuf::Message& val, bool mandatory);

	bool
	map();

	bool
	configure(google::protobuf::Message& message, bool mandatory);

	static bool
	configure(google::protobuf::Message& message, const boost::property_tree::ptree& config);

private:

	const boost::property_tree::ptree& p_;

	bool mandatoryCheck_;

};

template<typename T>
inline bool
PropertyMapper::add(std::string key, T& val, bool mandatory)
{
	auto value = p_.get_optional<T>(key);
	if (value)
	{
		APLOG_TRACE << "Property " << key << " = " << *value;
		val = *value;
		return true;
	}
	if (mandatory)
	{
		APLOG_ERROR << "PM: mandatory " << key << " missing";
		mandatoryCheck_ = false;
	}
	return false;
}

template<typename T>
inline bool
PropertyMapper::addVector(std::string key, std::vector<T>& val, bool mandatory)
{
	val.clear();
	auto value = p_.get_child_optional(key);
	if (value)
	{
		boost::property_tree::ptree child = *value;
		for (auto& it : child)
		{
			val.push_back(it.second.get_value<T>());
		}
		return true;
	}
	if (mandatory)
	{
		APLOG_ERROR << "PM: mandatory " << key << " missing";
		mandatoryCheck_ = false;
	}
	return false;
}

#endif /* UTIL_PROPERTYMAPPER_H_ */
