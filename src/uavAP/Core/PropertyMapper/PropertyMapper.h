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
#include <uavAP/Core/EnumMap.hpp>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include "uavAP/Core/Logging/APLogger.h"
#include <Eigen/Core>
#include <type_traits>

namespace boost
{
namespace posix_time
{
class time_duration;
}
}

class PropertyMapper
{
public:
	PropertyMapper(const boost::property_tree::ptree& p);

	template<typename PODType>
	bool
	add(const std::string& key, typename std::enable_if<std::is_pod<PODType>::value, PODType>::type& val,
			bool mandatory);


	template<typename Type>
	bool
	add(const std::string& key, typename std::enable_if<!std::is_pod<Type>::value, Type>::type& val,
			bool mandatory);

	template<typename T>
	bool
	addVector(const std::string& key, std::vector<T>& val, bool mandatory);

	bool
	addVector(const std::string&key, std::vector<boost::property_tree::ptree>& val, bool mandatory);

	bool
	add(const std::string& key, boost::posix_time::time_duration& val, bool mandatory);

	bool
	add(const std::string& key, std::string& val, bool mandatory);

	bool
	add(const std::string& key, Vector3& val, bool mandatory);

	bool
	add(const std::string& key, Vector2& val, bool mandatory);

	bool
	add(const std::string& key, boost::property_tree::ptree& val, bool mandatory);

	bool
	add(const std::string& key, google::protobuf::Message& val, bool mandatory);

	template <typename Type>
	bool
	add(const std::string& key, Eigen::Matrix<Type, Eigen::Dynamic, 1>& val, bool mandatory);

	template <typename Type>
	bool
	add(const std::string& key, Eigen::Array<Type, Eigen::Dynamic, 1>& val, bool mandatory);

	template <typename Enum>
	bool
	addEnum(const std::string& key, Enum& e, bool mandatory);

	template <typename Enum>
	bool
	addEnumVector(const std::string& key, std::vector<Enum>& e, bool mandatory);

	bool
	map();

	bool
	configure(google::protobuf::Message& message, bool mandatory);

	static bool
	configure(google::protobuf::Message& message, const boost::property_tree::ptree& config);

	PropertyMapper
	getChild(const std::string& key, bool mandatory);

	bool
	isEmpty() const;

private:

	const boost::property_tree::ptree& p_;

	bool mandatoryCheck_;

};

template<typename PODType>
inline bool
PropertyMapper::add(const std::string& key, typename std::enable_if<std::is_pod<PODType>::value, PODType>::type& val,
		bool mandatory)
{
	auto value = p_.get_optional<PODType>(key);
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

template<typename Type>
inline bool
PropertyMapper::add(const std::string& key, typename std::enable_if<!std::is_pod<Type>::value, Type>::type& val,
		bool mandatory)
{
	if (key.empty())
	{
		bool success = val.configure(p_);
		if (mandatory)
			mandatoryCheck_ &= success;

		return success;
	}
	else
	{
		boost::property_tree::ptree subtree;
		add(key, subtree, false);
		if (!subtree.empty())
		{
			if (val.configure(subtree))
				return true;
		}
		if (mandatory)
		{
			APLOG_ERROR << "PM: mandatory " << key << " missing";
			mandatoryCheck_ = false;
		}
		return false;
	}
}

template<typename T>
inline bool
PropertyMapper::addVector(const std::string& key, std::vector<T>& val, bool mandatory)
{
	val.clear();
	boost::optional<const boost::property_tree::ptree&> value;
	if (key.empty())
		value = p_;
	else
		value = p_.get_child_optional(key);
	if (value)
	{
		const boost::property_tree::ptree& config = *value;
		for (auto& it : config)
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

template<typename Type>
inline bool
PropertyMapper::add(const std::string& key, Eigen::Matrix<Type, Eigen::Dynamic, 1>& val,
		bool mandatory)
{
	auto value = p_.get_child_optional(key);
	if (value)
	{
		boost::property_tree::ptree child = *value;
		Eigen::Matrix<Type, -1, 1>  values(child.size(), 1);
		int k = 0;
		for (auto& it : child)
		{
			values[k++] = it.second.get_value<Type>();
		}
		val = values;
		return true;
	}
	if (mandatory)
	{
		APLOG_ERROR << "PM: mandatory " << key << " missing";
		mandatoryCheck_ = false;
	}
	return false;
}

template<typename Type>
inline bool
PropertyMapper::add(const std::string& key, Eigen::Array<Type, Eigen::Dynamic, 1>& val,
		bool mandatory)
{
	val = {};
	auto value = p_.get_child_optional(key);
	if (value)
	{
		boost::property_tree::ptree child = *value;
		Eigen::Matrix<Type, -1, 1> values(child.size(), 1);
		int k = 0;
		for (auto& it : child)
		{
			values[k++] = it.second.get_value<Type>();
		}
		val = values.array();
		return true;
	}
	if (mandatory)
	{
		APLOG_ERROR << "PM: mandatory " << key << " missing";
		mandatoryCheck_ = false;
	}
	return false;
}

template<typename Enum>
inline bool
PropertyMapper::addEnum(const std::string& key, Enum& e, bool mandatory)
{
	auto value = p_.get_optional<std::string>(key);
	if (value)
	{
		APLOG_TRACE << "Property " << key << " = " << *value;
		e = EnumMap<Enum>::convert(*value);
		return true;
	}
	if (mandatory)
	{
		APLOG_ERROR << "PM: mandatory " << key << " missing";
		mandatoryCheck_ = false;
	}
	return false;
}

template<typename Enum>
inline bool
PropertyMapper::addEnumVector(const std::string& key, std::vector<Enum>& e, bool mandatory)
{
	e.clear();
	auto value = p_.get_child_optional(key);
	if (value)
	{
		boost::property_tree::ptree child = *value;
		for (auto& it : child)
		{
			e.push_back(EnumMap<Enum>::convert(it.second.get_value<std::string>()));
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
