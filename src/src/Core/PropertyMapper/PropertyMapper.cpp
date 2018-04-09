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
 * PropertyMapper.cpp
 *
 *  Created on: Jun 17, 2017
 *      Author: mircot
 */

#include "uavAP/Core/PropertyMapper/PropertyMapper.h"

PropertyMapper::PropertyMapper(const boost::property_tree::ptree& p) :
		p_(p),
		mandatoryCheck_(true)
{
}

bool
PropertyMapper::add(std::string key, Duration& val, bool mandatory)
{
	auto value = p_.get_optional<int>(key);
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

bool
PropertyMapper::add(std::string key, boost::property_tree::ptree& val, bool mandatory)
{
	auto value = p_.get_child_optional(key);
	if (value)
	{
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

bool
PropertyMapper::add(std::string key, google::protobuf::Message& val, bool mandatory)
{
	auto value = p_.get_child_optional(key);
	if (value)
	{
		return configure(val, *value);
	}
	if (mandatory)
	{
		APLOG_ERROR << "PM: mandatory " << key << " missing";
		mandatoryCheck_ = false;
	}
	return false;
}

bool
PropertyMapper::addVector(std::string key, std::vector<boost::property_tree::ptree>& val,
		bool mandatory)
{
	val.clear();
	auto value = p_.get_child_optional(key);
	if (value)
	{
		boost::property_tree::ptree child = *value;
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

bool
PropertyMapper::map()
{
	//Do some error printouts
	return mandatoryCheck_;
}

template<typename Type>
boost::optional<Type>
get(const google::protobuf::FieldDescriptor* fdescriptor, const boost::property_tree::ptree& config)
{
	auto value = config.get_optional<Type>(fdescriptor->name());
	if (value)
	{
		APLOG_TRACE << "Property " << fdescriptor->name() << " = " << *value;
		return value;
	}
	if (!fdescriptor->has_default_value())
	{
		APLOG_ERROR << "PM: mandatory " << fdescriptor->name() << " missing";
	}

	return boost::none;
}

bool
PropertyMapper::configure(google::protobuf::Message& message, bool mandatory)
{
	if (configure(message, p_))
		return true;
	if (mandatory)
	{
		mandatoryCheck_ = false;
	}
	return false;
}

bool
PropertyMapper::configure(google::protobuf::Message& message,
		const boost::property_tree::ptree& config)
{
	using namespace google::protobuf;
	const Descriptor* descriptor = message.GetDescriptor();
	const Reflection* reflection = message.GetReflection();
	PropertyMapper pm(config);

	for (int i = 1; i <= descriptor->field_count(); ++i)
	{
		const FieldDescriptor* fDescriptor = descriptor->FindFieldByNumber(i);

		switch (fDescriptor->cpp_type())
		{
		case FieldDescriptor::CPPTYPE_MESSAGE:
			pm.add(fDescriptor->name(), *reflection->MutableMessage(&message, fDescriptor),
					!fDescriptor->has_default_value());
			break;
		case FieldDescriptor::CPPTYPE_BOOL:
			if (auto val = get<bool>(fDescriptor, config))
				reflection->SetBool(&message, fDescriptor, *val);
			break;
		case FieldDescriptor::CPPTYPE_DOUBLE:
			if (auto val = get<double>(fDescriptor, config))
				reflection->SetDouble(&message, fDescriptor, *val);
			break;
		case FieldDescriptor::CPPTYPE_FLOAT:
			if (auto val = get<float>(fDescriptor, config))
				reflection->SetFloat(&message, fDescriptor, *val);
			break;
		case FieldDescriptor::CPPTYPE_INT32:
			if (auto val = get<int32_t>(fDescriptor, config))
				reflection->SetInt32(&message, fDescriptor, *val);
			break;
		case FieldDescriptor::CPPTYPE_INT64:
			if (auto val = get<int64_t>(fDescriptor, config))
				reflection->SetInt64(&message, fDescriptor, *val);
			break;
		case FieldDescriptor::CPPTYPE_UINT32:
			if (auto val = get<uint32_t>(fDescriptor, config))
				reflection->SetUInt32(&message, fDescriptor, *val);
			break;
		case FieldDescriptor::CPPTYPE_UINT64:
			if (auto val = get<uint64_t>(fDescriptor, config))
				reflection->SetUInt64(&message, fDescriptor, *val);
			break;
		default:
			break;
		}
	}

	return true; //TODO Fix return value

}

