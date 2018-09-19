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
 * @file Factory.h
 * @brief Definition of the Factory class used for the different Factories in uavAP
 * @date Jul 26, 2017
 * @author Mirco Theile, mircot@illinois.edu
 */

#ifndef UAVAP_CORE_FRAMEWORK_FACTORY_H_
#define UAVAP_CORE_FRAMEWORK_FACTORY_H_
#include <boost/property_tree/ptree.hpp>
#include "uavAP/Core/Framework/FrameworkExceptions.h"
#include "uavAP/Core/Logging/APLogger.h"
#include <map>
#include <memory>

/**
 * @brief Template class for of a typical factory.
 *
 * The template type defines the Interface class which has to be implemented by all the classes that can
 * be created by the factory. The factory searches for the "type" argument in the configuration tree
 * to determine the class to create. In the constructor of any factory implementing this factory the
 * creatorMap_ has to be filled by registering creators using the addCreator function. If a Factory should
 * have a default class to create, the setDefault method should reference the string used for that specific
 * creator.
 */
template<class Type>
class Factory
{

public:

	virtual
	~Factory() = default;

	/**
	 * @brief 	Create function called by a helper. Uses "type" argument in config to determine the class to
	 * 			be created
	 * @param config Configuration tree
	 * @return Created object
	 */
	std::shared_ptr<Type>
	create(const boost::property_tree::ptree& config);

	using TypeId = const char* const;

protected:

	//!Defines the function that can be registered
	using Creator = std::function<std::shared_ptr<Type>(const boost::property_tree::ptree&) >;

	/**
	 * @brief Adds a creator function with a string id.
	 * @param id ID of that objects creator
	 * @param creator The objects creator that creates the object using a config tree
	 */
	template <class SpecificType>
	void
	addCreator();

	/**
	 * @brief Set a default id. If no type is set in the tree this default object is created.
	 * @param defaultId Default id.
	 */
	template <class SpecificType>
	void
	setDefault();

private:

	std::map<std::string, Creator> creatorMap_; //!< Map containing all the creators and their ids

	std::string defaultId_; //!< The default id

};

template<class Type>
inline std::shared_ptr<Type>
Factory<Type>::create(const boost::property_tree::ptree& config)
{
	if (creatorMap_.empty())
	{
		throw FactoryInitializationError("Creator Map is empty.");
	}

	std::string type;

	try
	{
		type = config.get<std::string>("type");
	} catch (boost::property_tree::ptree_error&)
	{
		if (defaultId_.empty())
			throw InvalidTypeError("Type missing in config.");

		type = defaultId_;
	}

	auto it = creatorMap_.find(type);

	if (it == creatorMap_.end())
	{
		throw InvalidTypeError("Unknown Type " + type + " in Factory.");
	}

	return it->second(config);
}

template <class Type>
template <class SpecificType>
inline void
Factory<Type>::addCreator()
{
	std::string type = SpecificType::typeId;
	if (creatorMap_.find(type) != creatorMap_.end())
	{
		APLOG_ERROR << "Same id for different creator added. Ignore.";
		return;
	}

	creatorMap_.insert(std::make_pair(type, &SpecificType::create));
}

template <class Type>
template <class SpecificType>
inline void
Factory<Type>::setDefault()
{
	defaultId_ = SpecificType::typeId;
	this->addCreator<SpecificType>();
}

#endif /* UAVAP_CORE_FRAMEWORK_FACTORY_H_ */
