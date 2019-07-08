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

#include <boost/property_tree/json_parser.hpp>
#include <uavAP/Core/LinearAlgebra.h>
#include <uavAP/Core/Time.h>
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"

using boost::property_tree::json_parser::read_json;

PropertyMapper::PropertyMapper(const Configuration& p) :
		p_(p), mandatoryCheck_(true)
{
}

bool
PropertyMapper::add(const std::string& key, std::string& val, bool mandatory)
{
	auto value = p_.get_optional<std::string>(key);
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


bool
PropertyMapper::add(const std::string& key, Duration& val, bool mandatory)
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
PropertyMapper::add(const std::string& key, boost::property_tree::ptree& val, bool mandatory)
{

	//See if it is a path that contains another configuration file
	auto path = p_.get_optional<std::string>(key);
	if (path && !path->empty())
	{
		try
		{
			boost::property_tree::ptree conf;
			boost::property_tree::read_json(*path, conf);
			val = conf;
			return true;

		}
		catch (boost::property_tree::json_parser::json_parser_error& err)
		{
			APLOG_DEBUG << "Cannot resolve string " << *path << " as path. " << err.what();
			//Do nothing
		}
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

bool
PropertyMapper::addVector(const std::string& key, std::vector<boost::property_tree::ptree>& val,
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

bool
PropertyMapper::add(const std::string& key, Vector3& val, bool mandatory)
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

bool
PropertyMapper::add(const std::string& key, Vector2& val, bool mandatory)
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

PropertyMapper
PropertyMapper::getChild(const std::string& key, bool mandatory)
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
	return PropertyMapper(boost::property_tree::ptree());
}

bool
PropertyMapper::isEmpty() const
{
	return p_.empty();
}
