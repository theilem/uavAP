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
 * @file FrameworkExceptions.h
 * @brief Defines the exceptions that can be thrown by the Framework helpers and factories
 * @date Jul 26, 2017
 * @author Mirco Theile, mircot@illinois.edu
 */

#ifndef UAVAP_CORE_FRAMEWORK_FRAMEWORKEXCEPTIONS_H_
#define UAVAP_CORE_FRAMEWORK_FRAMEWORKEXCEPTIONS_H_
#include <string>

/**
 * @brief general framework error
 */
class FrameworkError
{
public:

	FrameworkError(const std::string& what) :
			what_(what)
	{
	}

	std::string
	what()
	{
		return what_;
	}

private:

	std::string what_;
};

/**
 * @brief Data type invalid error
 */
class InvalidTypeError: public FrameworkError
{
public:
	InvalidTypeError(const std::string& what) :
			FrameworkError(what)
	{
	}
};

/**
 * @brief Error in the factory initialization
 */
class FactoryInitializationError: public FrameworkError
{
public:
	FactoryInitializationError(const std::string& what) :
			FrameworkError(what)
	{
	}
};

#endif /* UAVAP_CORE_FRAMEWORK_FRAMEWORKEXCEPTIONS_H_ */
