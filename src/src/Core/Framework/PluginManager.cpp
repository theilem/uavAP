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
 * @file PluginManager.cpp
 * @date Sep 20, 2018
 * @author Mirco Theile, mirco.theile@tum.de
 * @brief
 */
#include <dlfcn.h>

#include <uavAP/Core/PropertyMapper/PropertyMapper.h>
#include "uavAP/Core/Framework/PluginManager.h"

bool
PluginManager::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	boost::property_tree::ptree plugins;
	pm.add("plugins", plugins, true);

	for (const auto& it : plugins)
	{
		auto path = it.second.get_value<std::string>();
		addPlugin(path);
	}
	return pm.map();
}

bool
PluginManager::addPlugin(const std::string& path)
{
	auto handle = loadPlugin(path);

	if (!handle)
	{
		APLOG_ERROR << "Cannot load shared object at: " << path;
		return false;
	}

	if (!registerCreators(handle))
	{
		APLOG_ERROR << "Cannot register creators for shared object at: " << path;
		return false;
	}

	return true;
}

PluginManager::PluginHandle
PluginManager::loadPlugin(const std::string& path)
{
	APLOG_DEBUG << "Opening shared object from: " << path;
	return dlopen(path.data() , RTLD_NOW);
}

bool
PluginManager::registerCreators(PluginHandle handle)
{
	void* registerPlugin = dlsym(handle, "register_plugin");

	if (registerPlugin == nullptr)
		return false;

	FunctionPtr func = reinterpret_cast<FunctionPtr>(registerPlugin);
	func();

	return true;

}
