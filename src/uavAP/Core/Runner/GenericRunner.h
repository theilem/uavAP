/*
 * GenericRunner.h
 *
 *  Created on: Jun 13, 2019
 *      Author: mirco
 */

#ifndef UAVAP_CORE_RUNNER_GENERICRUNNER_H_
#define UAVAP_CORE_RUNNER_GENERICRUNNER_H_

#include <uavAP/Core/Logging/APLogger.h>
#include "uavAP/Core/Runner/IRunnableObject.h"

template<class Aggregator>
class GenericRunner
{

public:

	GenericRunner(Aggregator& agg);

	bool
	runStage(RunStage stage);

	bool
	runAllStages();

private:

	Aggregator& agg_;
};

template<class Aggregator>
inline
GenericRunner<Aggregator>::GenericRunner(Aggregator& agg) :
		agg_(agg)
{
}

template<class Aggregator>
inline bool
GenericRunner<Aggregator>::runStage(RunStage stage)
{
	auto runnableObjects = agg_.template getAll<IRunnableObject>();
	bool error = false;
	for (auto it : runnableObjects)
	{
		if (it->run(stage))
			error = true;
	}
	return error;
}

template<class Aggregator>
inline bool
GenericRunner<Aggregator>::runAllStages()
{
	APLOG_DEBUG << "Run stage INIT";
	if (runStage(RunStage::INIT))
	{
		APLOG_ERROR << "Errors occured in RunStage INIT";
		return true;
	}
	APLOG_DEBUG << "Run stage NORMAL";
	if (runStage(RunStage::NORMAL))
	{
		APLOG_ERROR << "Errors occured in RunStage NORMAL";
		return true;
	}
	APLOG_DEBUG << "Run stage FINAL";
	if (runStage(RunStage::FINAL))
	{
		APLOG_ERROR << "Errors occured in RunStage FINAL";
		return true;
	}
	APLOG_DEBUG << "Run stages finished successfull";
	return false;
}

#endif /* UAVAP_CORE_RUNNER_GENERICRUNNER_H_ */
