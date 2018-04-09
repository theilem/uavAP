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
 * SynchronizedRunner.h
 *
 *  Created on: Jul 1, 2017
 *      Author: mircot
 */

#ifndef UAVAP_CORE_RUNNER_SYNCHRONIZEDRUNNER_H_
#define UAVAP_CORE_RUNNER_SYNCHRONIZEDRUNNER_H_
#include <boost/interprocess/shared_memory_object.hpp>

class Aggregator;
struct Synchronizer;
class SynchronizedRunner
{
public:

	SynchronizedRunner();

	bool
	runSynchronized(Aggregator& agg);

private:

	boost::interprocess::shared_memory_object sync_;

};

#endif /* UAVAP_CORE_RUNNER_SYNCHRONIZEDRUNNER_H_ */
