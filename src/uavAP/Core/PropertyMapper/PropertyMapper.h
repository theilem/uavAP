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

#ifndef UAVAP_CORE_PROPERTYMAPPER_PROPERTYMAPPER_H_
#define UAVAP_CORE_PROPERTYMAPPER_PROPERTYMAPPER_H_

#include <uavAP/Core/EnumMap.hpp>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/PropertyMapper/Parameter.h>
#include <uavAP/Core/PropertyMapper/ParameterRef.h>
#include <uavAP/Core/Time.h>
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

template<typename Configuration>
class PropertyMapper
{
public:
	PropertyMapper(const Configuration& p);

	template<typename PODType>
	bool
	add(const std::string& key,
			typename std::enable_if<std::is_pod<PODType>::value, PODType>::type& val,
			bool mandatory);

	template<typename Type>
	bool
	add(const std::string& key, typename std::enable_if<!std::is_pod<Type>::value, Type>::type& val,
			bool mandatory);

	template<typename T>
	bool
	addVector(const std::string& key, std::vector<T>& val, bool mandatory);

	bool
	addVector(const std::string&key, std::vector<Configuration>& val, bool mandatory);

	bool
	add(const std::string& key, Duration& val, bool mandatory);

	bool
	add(const std::string& key, std::string& val, bool mandatory);

	bool
	add(const std::string& key, Vector3& val, bool mandatory);

	bool
	add(const std::string& key, Vector2& val, bool mandatory);

	bool
	add(const std::string& key, Configuration& val, bool mandatory);

	template<typename Type>
	bool
	add(const std::string& key, Eigen::Matrix<Type, Eigen::Dynamic, 1>& val, bool mandatory);

	template<typename Type>
	bool
	add(const std::string& key, Eigen::Array<Type, Eigen::Dynamic, 1>& val, bool mandatory);

	template<typename Enum>
	bool
	addEnum(const std::string& key, Enum& e, bool mandatory);

	template<typename Enum>
	bool
	addEnumVector(const std::string& key, std::vector<Enum>& e, bool mandatory);

	template<typename Param>
	typename std::enable_if<
			!(is_parameter_set<typename Param::ValueType>::value
					|| is_parameter_set_ref<typename Param::ValueType>::value), bool>::type
	operator&(Param& param);

	template<typename Param>
	typename std::enable_if<is_parameter_set<typename Param::ValueType>::value, bool>::type
	operator&(Param& param);

	template<typename Param>
	typename std::enable_if<is_parameter_set_ref<typename Param::ValueType>::value, bool>::type
	operator&(Param& param);

	bool
	map();

	PropertyMapper
	getChild(const std::string& key, bool mandatory);

	bool
	isEmpty() const;

protected:

	const Configuration& p_;
	bool mandatoryCheck_;

private:
	template<typename Type>
	struct is_special_param
	{
		template<typename _1> static char
		chk(
				decltype(std::declval<PropertyMapper<Configuration>>().add(std::string(), std::declval<_1&>(), true)));
		template<typename > static int
		chk(...);

		static constexpr bool value = sizeof(chk<Type>(0)) == sizeof(char);
	};

	template<typename Type>
	bool
	addSpecific(const std::string& key,
			typename std::enable_if<is_special_param<Type>::value, Type>::type& val,
			bool mandatory);

	template<typename Type>
	bool
	addSpecific(const std::string& key,
			typename std::enable_if<std::is_enum<Type>::value, Type>::type& val,
			bool mandatory);

	template<typename Type>
	bool
	addSpecific(const std::string& key,
			typename std::enable_if<!is_special_param<Type>::value && !std::is_enum<Type>::value, Type>::type& val,
			bool mandatory);

};

template<typename Config>
template<typename PODType>
inline bool
PropertyMapper<Config>::add(const std::string& key,
		typename std::enable_if<std::is_pod<PODType>::value, PODType>::type& val, bool mandatory)
{
	auto value = p_.template get_optional<PODType>(key);
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

template<typename Config>
template<typename Type>
inline bool
PropertyMapper<Config>::add(const std::string& key,
		typename std::enable_if<!std::is_pod<Type>::value, Type>::type& val, bool mandatory)
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
		Config subtree;
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

template<typename Config>
template<typename T>
inline bool
PropertyMapper<Config>::addVector(const std::string& key, std::vector<T>& val, bool mandatory)
{
#ifndef ERIKA
	val.clear();
	boost::optional<const Config&> value;
	if (key.empty())
		value = p_;
	else
		value = p_.get_child_optional(key);
	if (value)
	{
		const Config& config = *value;
		for (auto& it : config)
		{
			val.push_back(it.second.template get_value<T>());
		}
		return true;
	}
#endif
	if (mandatory)
	{
		APLOG_ERROR << "PM: mandatory " << key << " missing";
		mandatoryCheck_ = false;
	}
	return false;
}

template<typename Config>
template<typename Type>
inline bool
PropertyMapper<Config>::add(const std::string& key, Eigen::Matrix<Type, Eigen::Dynamic, 1>& val,
		bool mandatory)
{
	auto value = p_.get_child_optional(key);
	if (value)
	{
		Config child = *value;
		Eigen::Matrix<Type, -1, 1> values(child.size(), 1);
		int k = 0;
		for (auto& it : child)
		{
			values[k++] = it.second.template get_value<Type>();
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

template<typename Config>
template<typename Type>
inline bool
PropertyMapper<Config>::add(const std::string& key, Eigen::Array<Type, Eigen::Dynamic, 1>& val,
		bool mandatory)
{
	val =
	{};
	auto value = p_.get_child_optional(key);
	if (value)
	{
		Config child = *value;
		Eigen::Matrix<Type, -1, 1> values(child.size(), 1);
		int k = 0;
		for (auto& it : child)
		{
			values[k++] = it.second.template get_value<Type>();
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

template<typename Config>
template<typename Enum>
inline bool
PropertyMapper<Config>::addEnum(const std::string& key, Enum& e, bool mandatory)
{
	auto value = p_.template get_optional<std::string>(key);
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

template<typename Config>
template<typename Enum>
inline bool
PropertyMapper<Config>::addEnumVector(const std::string& key, std::vector<Enum>& e, bool mandatory)
{
	e.clear();
	auto value = p_.get_child_optional(key);
	if (value)
	{
		Config child = *value;
		for (auto& it : child)
		{
			e.push_back(EnumMap<Enum>::convert(it.second.template get_value<std::string>()));
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

template<typename Config>
PropertyMapper<Config>::PropertyMapper(const Config& p) :
		p_(p), mandatoryCheck_(true)
{
}

template<typename Config>
bool
PropertyMapper<Config>::add(const std::string& key, std::string& val, bool mandatory)
{
	auto value = p_.template get_optional<std::string>(key);
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

template<typename Config>
bool
PropertyMapper<Config>::add(const std::string& key, Duration& val, bool mandatory)
{
	auto value = p_.template get_optional<int>(key);
	if (value)
	{
		APLOG_TRACE << "Property " << key << " = " << *value;
		val = Milliseconds(*value);
		return true;
	}
	if (mandatory)
	{
		APLOG_ERROR << "PM: mandatory " << key << " missing";
		mandatoryCheck_ = false;
	}
	return false;
}

template<typename Config>
bool
PropertyMapper<Config>::add(const std::string& key, Config& val, bool mandatory)
{

	//See if it is a path that contains another configuration file
	auto path = p_.template get_optional<std::string>(key);
	if (path && !path->empty())
	{
//		try
//		{
//			boost::property_tree::ptree conf;
//			boost::property_tree::read_json(*path, conf);
//			val = conf;
//			return true;
//
//		}
//		catch (boost::property_tree::json_parser::json_parser_error& err)
//		{
//			APLOG_DEBUG << "Cannot resolve string " << *path << " as path. " << err.what();
//			//Do nothing
//		}
	}
	else
	{
		auto value = p_.get_child_optional(key);
		if (value)
		{
			val = *value;
			return true;
		}
	}
	if (mandatory)
	{
		APLOG_ERROR << "PM: mandatory " << key << " missing";
		mandatoryCheck_ = false;
	}
	return false;
}

template<typename Config>
bool
PropertyMapper<Config>::addVector(const std::string& key, std::vector<Config>& val, bool mandatory)
{
	val.clear();
	auto value = p_.get_child_optional(key);
	if (value)
	{
		Config child = *value;
		for (auto& it : child)
		{
			val.push_back(it.second);
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

template<typename Config>
bool
PropertyMapper<Config>::map()
{
	//Do some error printouts
	return mandatoryCheck_;
}

template<typename Config>
bool
PropertyMapper<Config>::add(const std::string& key, Vector3& val, bool mandatory)
{
	std::vector<double> vec;
	if (!addVector(key, vec, mandatory))
		return false;

	if (vec.size() == 3)
	{
		val = Vector3(vec[0], vec[1], vec[2]);
		return true;
	}
	if (mandatory)
	{
		APLOG_ERROR << "PM: Vector " << key << " does not have 3 values, only " << vec.size();
		mandatoryCheck_ = false;
	}
	return false;
}

template<typename Config>
bool
PropertyMapper<Config>::add(const std::string& key, Vector2& val, bool mandatory)
{
	std::vector<double> vec;
	if (!addVector(key, vec, mandatory))
		return false;

	if (vec.size() == 2)
	{
		val = Vector2(vec[0], vec[1]);
		return true;
	}
	if (mandatory)
	{
		APLOG_ERROR << "PM: Vector " << key << " does not have 2 values, only " << vec.size();
		mandatoryCheck_ = false;
	}
	return false;
}

template<typename Config>
PropertyMapper<Config>
PropertyMapper<Config>::getChild(const std::string& key, bool mandatory)
{

	auto value = p_.get_child_optional(key);
	if (value)
	{
		PropertyMapper p(*value);
		return p;
	}
	if (mandatory)
	{
		APLOG_ERROR << "PM: mandatory " << key << " missing";
		mandatoryCheck_ = false;
	}
	return PropertyMapper(Config());
}

template<typename Config>
bool
PropertyMapper<Config>::isEmpty() const
{
	return p_.empty();
}

template<typename Config>
template<typename Param>
inline typename std::enable_if<!(is_parameter_set<typename Param::ValueType>::value
		|| is_parameter_set_ref<typename Param::ValueType>::value), bool>::type
PropertyMapper<Config>::operator &(Param& param)
{
	return addSpecific<typename Param::ValueType>(param.id, param.value, param.mandatory);
}

template<typename Config>
template<typename Param>
inline typename std::enable_if<is_parameter_set<typename Param::ValueType>::value, bool>::type
PropertyMapper<Config>::operator &(Param& param)
{
	auto pm = getChild(param.id, param.mandatory);
	if (!pm.isEmpty())
	{
		param.value.configure(pm);
		return pm.map();
	}
	return false;
}

template<typename Config>
template<typename Param>
inline typename std::enable_if<is_parameter_set_ref<typename Param::ValueType>::value, bool>::type
PropertyMapper<Config>::operator &(Param& param)
{
	auto pm = getChild(param.id, param.mandatory);
	if (!pm.isEmpty())
	{
		configure(pm, param.value);
		return pm.map();
	}
	return false;
}

template<typename Config>
template<typename Type>
bool
PropertyMapper<Config>::addSpecific(const std::string& key,
		typename std::enable_if<is_special_param<Type>::value, Type>::type& val, bool mandatory)
{
	return add(key, val, mandatory);
}

template<typename Config>
template<typename Type>
bool
PropertyMapper<Config>::addSpecific(const std::string& key,
		typename std::enable_if<std::is_enum<Type>::value, Type>::type& val, bool mandatory)
{
	return addEnum(key, val, mandatory);
}

template<typename Config>
template<typename Type>
bool
PropertyMapper<Config>::addSpecific(const std::string& key,
		typename std::enable_if<!is_special_param<Type>::value && !std::is_enum<Type>::value, Type>::type& val, bool mandatory)
{
	return add<Type>(key, val, mandatory);
}

#endif /* UAVAP_CORE_PROPERTYMAPPER_PROPERTYMAPPER_H_ */
