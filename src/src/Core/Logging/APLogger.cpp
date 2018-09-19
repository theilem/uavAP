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
 * APLogger.cpp
 *
 *  Created on: Jun 26, 2017
 *      Author: mircot
 */

#include "uavAP/Core/Logging/APLogger.h"

APLogger* APLogger::instance_ = nullptr;

APLogger*
APLogger::instance()
{
	static CGuard g;
	if (!instance_)
		instance_ = new APLogger;
	return instance_;
}

void
APLogger::setLogLevel(LogLevel level)
{
	setLevel_ = level;
}

void
APLogger::setSink(std::ostream& sink)
{
	sink_.rdbuf(sink.rdbuf());
}

std::ostream&
APLogger::log(LogLevel level)
{
	if (level >= setLevel_)
	{
		sink_ << std::endl;
		if (!moduleName_.empty())
			sink_ << "[" << moduleName_ << "]";
		return sink_;
	}
	return emptySink_;
}

std::ostream&
APLogger::log(LogLevel level, const std::string& module)
{
	if (level >= setLevel_ && moduleName_.compare(module) == 0)
	{
		sink_ << std::endl;
		if (!moduleName_.empty())
			sink_ << "[" << moduleName_ << "]";
		return sink_;
	}
	return emptySink_;
}

APLogger::APLogger() :
		setLevel_(LogLevel::WARN), sink_(nullptr), emptySink_(nullptr)
{
	sink_.rdbuf(std::cout.rdbuf());
}

APLogger::CGuard::~CGuard()
{
	if (APLogger::instance_ != nullptr)
	{
		std::cout << std::endl; //Quick printout fix
		delete APLogger::instance_;
		APLogger::instance_ = nullptr;
	}
}

void
APLogger::setModuleName(std::string name)
{
	moduleName_ = name;
}
