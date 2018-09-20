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
 * Helper.cpp
 *
 *  Created on: Jul 26, 2017
 *      Author: mircot
 */
#include <boost/optional/optional.hpp>
#include <boost/property_tree/json_parser.hpp>
#include "uavAP/Core/Framework/PluginManager.h"
#include "uavAP/Core/Framework/Helper.h"
#include "uavAP/Core/Object/Aggregator.h"

Aggregator
Helper::createAggregation(const std::string& configPath)
{
	boost::property_tree::ptree conf;
	if (!configPath.empty())
	{
		boost::property_tree::read_json(configPath, conf);
	}

	try
	{
		auto globalConf = conf.get_child("global");
		mergeGlobalConfig(conf, globalConf);
	} catch (boost::property_tree::ptree_error&)
	{
		//No global config
	}

	return createAggregation(conf);
}

Aggregator
Helper::createAggregation(const boost::property_tree::ptree& conf)
{
	Aggregator agg;

	PluginManager plugin;
	auto child = conf.get_child_optional("plugin_manager");
	if (child)
	{
		plugin.configure(*child);
	}

	for (auto& it : conf)
	{

		//Remove from default factories
		defaultCreators_.erase(it.first);

		auto creator = creators_.find(it.first);
		if (creator == creators_.end())
		{
			APLOG_ERROR << "Factory for " << it.first << " not found. Ignore.";
			continue;
		}

		try
		{
			APLOG_TRACE << "Creating and configuring " << it.first;
			auto obj = creator->second(it.second);
			agg.add(obj);
			APLOG_TRACE << "Added " << it.first << " to aggregation";
		} catch (FrameworkError& err)
		{
			APLOG_ERROR << "Exception: " << err.what();
		}
	}

	for (auto& it : defaultCreators_)
	{
		try
		{
			auto obj = it.second();
			agg.add(obj);
		} catch (FrameworkError& err)
		{
			APLOG_ERROR << "Exception: " << err.what();
		}
	}

	return agg;
}

void
Helper::mergeGlobalConfig(boost::property_tree::ptree& config,
		const boost::property_tree::ptree& globalConf)
{
	for (auto& confIt : config)
	{
		for (auto& it : globalConf)
		{
			confIt.second.push_back(it);
		}
	}
}
