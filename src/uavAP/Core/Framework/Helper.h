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
 * @file Helper.h
 * @brief Defines the Helper class which is used for the fast setup of any process.
 * @date Jul 26, 2017
 * @author Mirco Theile, mirco.theile@tum.de
 */

#ifndef UAVAP_CORE_FRAMEWORK_HELPER_H_
#define UAVAP_CORE_FRAMEWORK_HELPER_H_
#include <boost/property_tree/ptree.hpp>
#include "uavAP/Core/Framework/Factory.h"
#include "uavAP/Core/Object/IAggregatableObject.h"
#include <functional>
#include <map>
#include <string>

/**
 * @brief Helper class using a configuration to create an Aggregation.
 *
 * To use a Helper, factories have to be registered by calling the add* functions. Therefore, the best way to do
 * that is to inherit from the Helper class and register the factories in the default constructor.
 */
class Helper
{
public:

	/**
	 * @brief Create an Aggregator containing Objects defined in a configuration loaded in from a config path.
	 * @param configPath path to the configuration .json file
	 * @return Aggregator containing the objects
	 */
	Aggregator
	createAggregation(const std::string& configPath);

	/**
	 * @brief Create an Aggregator containing Objects defined in a configuration.
	 * @param config Configuration object tree
	 * @return Aggregator containing the objects
	 */
	Aggregator
	createAggregation(const Configuration& config);

	void
	setDefaultPluginRestriction(PluginRestriction plug);

protected:

	/**
	 * @brief Adds a factory to the possible factories. The template argument defines the factory.
	 * @param id ID of the factory
	 */
	template<class FactoryType>
	void
	addFactory(PluginRestriction restriction = PluginRestriction::DEFAULT);

	/**
	 * @brief Adds a creator without a factory. The creator should create an Aggregatable object.
	 * @param id ID of the Creator object
	 */
	template<class Aggregatable>
	void
	addCreator();

	/**
	 * @brief Add a default factory that is always used.
	 *
	 * If the configuration file indicates another type that is not the default type of the factory, the configuration
	 * file defined object will be created.
	 * @param id ID of the factory
	 */
	template<class FactoryType>
	void
	addDefault(PluginRestriction restriction = PluginRestriction::DEFAULT);

	/**
	 * @brief Add a default creator that does not need a factory and will be created by default.
	 * @param id ID of the creator
	 */
	template<class Aggregatable>
	void
	addDefaultCreator();



	template<class Configurable>
	void
	addConfigurable();

private:

	/**
	 * @brief Creates an IAggregatableObject using a factory and a configuration tree
	 * @param factory Factory to be used to create the object
	 * @param config Configuration object passed to the factory
	 * @return Object created from the factory
	 */
	template<class FactoryType>
	static std::shared_ptr<IAggregatableObject>
	createAggregatable(FactoryType factory, const Configuration& config);


	template<class Configurable>
	static std::shared_ptr<IAggregatableObject>
	createConfigurable(const Configuration& config);

	/**
	 * @brief Merging to configuration trees, adding global configuration subtrees to the configuration tree
	 * @param config Configuration tree to be appended with the globalConf
	 * @param globalConf Global configuration to be appended to the config
	 */
	void
	mergeGlobalConfig(boost::property_tree::ptree& config,
			const Configuration& globalConf);

	//! Creator functor to create an IAggregatableObject
	using CreatorAgg = std::function<std::shared_ptr<IAggregatableObject>(const Configuration&)>;

	//! Map containing the Creators that do not need factories mapped to their ID.
	std::map<std::string, CreatorAgg> creators_;

	//! Default creator functor to create an IAggregatableObject that does not need a configuration
	using DefaultCreatorAgg = std::function<std::shared_ptr<IAggregatableObject>()>;

	//! Map containing the Default Creators that do not need factories mapped to their ID.
	std::map<std::string, DefaultCreatorAgg> defaultCreators_;

	PluginRestriction defaultPluginRestriction_  = PluginRestriction::NOT_ALLOWED;

};

template<class FactoryType>
inline void
Helper::addFactory(PluginRestriction restriction)
{
	std::string type = FactoryType::typeId();
	if (creators_.find(type) != creators_.end())
	{
		APLOG_ERROR << "Same id for different factory added. Ignore.";
		return;
	}

	if (restriction == PluginRestriction::DEFAULT)
		restriction = defaultPluginRestriction_;
	FactoryType factory;
	factory.setPluginRestriction(restriction);
	CreatorAgg creator = std::bind(&Helper::createAggregatable<FactoryType>, factory,
			std::placeholders::_1);

	creators_.insert(std::make_pair(type, creator));
}

template<class FactoryType>
inline void
Helper::addDefault(PluginRestriction restriction)
{
	std::string type = FactoryType::typeId();
	if (defaultCreators_.find(type) != defaultCreators_.end())
	{
		APLOG_ERROR << "Same id for different default factory added. Ignore.";
		return;
	}

	FactoryType factory;
	boost::property_tree::ptree emptyConf;
	DefaultCreatorAgg defaultCreator = std::bind(&Helper::createAggregatable<FactoryType>, factory,
			emptyConf);

	CreatorAgg creator = std::bind(&Helper::createAggregatable<FactoryType>, factory,
			std::placeholders::_1);

	defaultCreators_.insert(std::make_pair(type, defaultCreator));
	creators_.insert(std::make_pair(type, creator));
}

template<class FactoryType>
inline std::shared_ptr<IAggregatableObject>
Helper::createAggregatable(FactoryType factory, const Configuration& config)
{
	auto obj = factory.create(config);
	if (auto aggObj = std::dynamic_pointer_cast<IAggregatableObject>(obj))
	{
		return aggObj;
	}
	throw InvalidTypeError("Object is not aggregatable.");
}

template<class Aggregatable>
inline void
Helper::addCreator()
{

	std::string type = Aggregatable::typeId;
	if (creators_.find(type) != creators_.end())
	{
		APLOG_ERROR << "Same id for different factory added. Ignore.";
		return;
	}

	CreatorAgg creator = &Aggregatable::create;

	creators_.insert(std::make_pair(type, creator));
}

template<class Aggregatable>
inline void
Helper::addDefaultCreator()
{
	std::string type = Aggregatable::typeId;
	if (defaultCreators_.find(type) != defaultCreators_.end())
	{
		APLOG_ERROR << "Same id for different default factory added. Ignore.";
		return;
	}

	boost::property_tree::ptree emptyConf;
	DefaultCreatorAgg defaultCreator = std::bind(&Aggregatable::create, emptyConf);
	defaultCreators_.insert(std::make_pair(type, defaultCreator));

	CreatorAgg creator = &Aggregatable::create;
	creators_.insert(std::make_pair(type, creator));
}

template<class Configurable>
inline void
Helper::addConfigurable()
{
	std::string type = Configurable::typeId;
	if (creators_.find(type) != creators_.end())
	{
		APLOG_ERROR << "Same id for different configurable added. Ignore.";
		return;
	}

	CreatorAgg creator = &Helper::createConfigurable<Configurable>;

	creators_.insert(std::make_pair(type, creator));
}

template<class Configurable>
inline std::shared_ptr<IAggregatableObject>
Helper::createConfigurable(const Configuration& config)
{
	auto obj = std::make_shared<Configurable>();
	if (!obj->configure(config))
	{
		APLOG_ERROR << Configurable::typeId << ": configuration failed.";
	}
	return obj;
}

#endif /* UAVAP_CORE_FRAMEWORK_HELPER_H_ */
