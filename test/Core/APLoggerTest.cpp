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
 * APLoggerTest.cpp
 *
 *  Created on: Jun 24, 2017
 *      Author: mircot
 */

#include <boost/test/unit_test.hpp>

#include "uavAP/Core/Logging/APLogger.h"

BOOST_AUTO_TEST_SUITE(LoggerTests)

BOOST_AUTO_TEST_CASE(LoggingTest)
{
	std::ostream sink(nullptr);
	std::ostringstream stream;

	sink.rdbuf(stream.rdbuf());
	APLogger::instance()->setSink(sink);
	APLogger::instance()->setLogLevel(LogLevel::DEBUG);

	APLOG_ERROR << "error";
	APLOG_WARN << "warn";
	APLOG_DEBUG << "debug";
	APLOG_TRACE << "trace";
	BOOST_CHECK_EQUAL(stream.str().compare("
[ERROR] error
[WARNING] warn
[DEBUG] debug"), 0);
	stream.str("");

	APLogger::instance()->setLogLevel(LogLevel::NONE);

	APLOG_ERROR << "error";
	APLOG_WARN << "warn";
	APLOG_DEBUG << "debug";
	APLOG_TRACE << "trace";
	BOOST_CHECK_EQUAL(stream.str().compare(""), 0);
	stream.str("");

	APLogger::instance()->setLogLevel(LogLevel::TRACE);

	APLOG_ERROR << "error";
	APLOG_WARN << "warn";
	APLOG_DEBUG << "debug";
	APLOG_TRACE << "trace";

	BOOST_CHECK_EQUAL(
			stream.str().compare("
[ERROR] error
[WARNING] warn
[DEBUG] debug
[TRACE] trace"),
			0);
	stream.str("");

	APLogger::instance()->setSink(std::cout);
	APLogger::instance()->setLogLevel(LogLevel::WARN);
}

BOOST_AUTO_TEST_SUITE_END()
