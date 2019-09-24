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
 * APLogger.h
 *
 *  Created on: Jun 23, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_LOGGING_APLOGGER_H_
#define UAVAP_CORE_LOGGING_APLOGGER_H_

#include <ostream>
#include <string>

enum class LogLevel
{
	TRACE = 0, DEBUG = 1, WARN = 2, ERROR = 3, NONE = 4
};

class APLogger
{
public:

	static APLogger*
	instance();

	void
	setLogLevel(LogLevel level);

	void
	setSink(std::ostream& sink);

	void
	setModuleName(const std::string& name);

	std::ostream&
	log(LogLevel level);

	std::ostream&
	log(LogLevel level, const std::string& module);

	void
	flush();

private:

	static APLogger* instance_;

	LogLevel setLevel_;
	std::ostream sink_;
	std::ostream emptySink_;

	std::string moduleName_;

	bool isFlushed_ = true;

	APLogger();

	class CGuard
	{
	public:
		~CGuard();
	};
};

#define APLOG(level) (APLogger::instance()->log(level))
#ifdef NO_LOGGING
#define APLOG_ERROR if (0) APLOG(::LogLevel::ERROR)
#define APLOG_WARN if (0) APLOG(::LogLevel::WARN)
#define APLOG_DEBUG if (0) APLOG(::LogLevel::DEBUG)
#define APLOG_TRACE if (0) APLOG(::LogLevel::TRACE)

#define MODULE_LOG_ERROR(module) if (0) MODULE_LOG(::LogLevel::ERROR, module)
#define MODULE_LOG_WARN(module) if (0) MODULE_LOG(::LogLevel::WARN, module)
#define MODULE_LOG_DEBUG(module) if (0) MODULE_LOG(::LogLevel::DEBUG, module)
#define MODULE_LOG_TRACE(module) if (0) MODULE_LOG(::LogLevel::TRACE, module)

#else

#define APLOG_ERROR (APLOG(::LogLevel::ERROR) << "[ERROR] ")
#define APLOG_WARN (APLOG(::LogLevel::WARN) << "[WARNING] ")

#ifdef NODEBUG
#define APLOG_DEBUG if (0) APLOG(::LogLevel::DEBUG)
#define APLOG_TRACE if (0) APLOG(::LogLevel::TRACE)
#else
#define APLOG_DEBUG (APLOG(::LogLevel::DEBUG) << "[DEBUG] ")
#define APLOG_TRACE (APLOG(::LogLevel::TRACE) << "[TRACE] ")
#endif

#define MODULE_LOG(level, module) (APLogger::instance()->log(level, module))
#define MODULE_LOG_ERROR(module) (MODULE_LOG(::LogLevel::ERROR, module) << "[ERROR] ")
#define MODULE_LOG_WARN(module) (MODULE_LOG(::LogLevel::WARN, module) << "[WARNING] ")

#ifdef NODEBUG
#define MODULE_LOG_DEBUG(module) if (0) MODULE_LOG(::LogLevel::DEBUG, module)
#define MODULE_LOG_TRACE(module) if (0) MODULE_LOG(::LogLevel::TRACE, module)
#else
#define MODULE_LOG_DEBUG(module) (MODULE_LOG(::LogLevel::DEBUG, module) << "[DEBUG] ")
#define MODULE_LOG_TRACE(module) (MODULE_LOG(::LogLevel::TRACE, module) << "[TRACE] ")
#endif

#endif //NO_LOGGING

#endif /* UAVAP_CORE_LOGGING_APLOGGER_H_ */
