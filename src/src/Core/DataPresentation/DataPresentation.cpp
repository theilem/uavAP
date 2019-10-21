/*
 * DataPresentation.cpp
 *
 *  Created on: Jul 8, 2019
 *      Author: mirco
 */
#include <uavAP/Core/DataPresentation/DataPresentation.h>
#include <uavAP/Core/Logging/APLogger.h>
#include <uavAP/Core/Object/AggregatableObjectImpl.hpp>

std::shared_ptr<DataPresentation>
DataPresentation::create(const Configuration& config)
{
	auto dp = std::make_shared<DataPresentation>();
	if (!dp->configure(config))
	{
		APLOG_ERROR << "DataPresentation configuration failed";
	}
	return dp;

}
