/*
 *
 */
#include "uavAP/API/ap_ext/ap_ext.h"
#include "uavAP/API/ap_ext/ApExtManager.h"
#include "uavAP/API/ap_ext/ApExtHelper.h"
#include <string>
#include <cpsCore/Synchronization/SimpleRunner.h>

#define SERVO_MIN 3600
#define SERVO_MAX 8400
#define SERVO_RANGE_HALF 2400

static Aggregator aggregator;
static std::string configPath = "/usr/local/config/alvolo.json";

int
ap_ext_setup()
{
	CPSLOG_DEBUG << "Setup of ap_ext";
	if (!aggregator.empty())
	{
		CPSLOG_ERROR << "Setup already executed.";
		return -1;
	}

	aggregator = ApExtHelper::createAggregation(configPath);

	SimpleRunner runner(aggregator);
	if (runner.runAllStages())
	{
		CPSLOG_ERROR << "Running aggregator stages failed";
		return -1;
	}

	return 0;
}

int
ap_ext_sense(const struct data_sample_t* sample)
{
	auto apManager = aggregator.getOne<ApExtManager>();
	if (!apManager)
	{
		CPSLOG_ERROR << "Setup not called before sensing.";
		return -1;
	}

	return apManager->ap_sense(sample);
}

int
ap_ext_actuate(unsigned long* pwm, unsigned int num_channels)
{
	auto apManager = aggregator.getOne<ApExtManager>();
	if (!apManager)
	{
		CPSLOG_ERROR << "Setup not called before actuating.";
		return -1;
	}

	return apManager->ap_actuate(pwm, num_channels);
}

int
ap_ext_teardown()
{
	CPSLOG_DEBUG << "Ap_ext teardown called";
	aggregator.cleanUp();
	return 0;
}

int
ap_ext_ctrl(int* cmd)
{
	return 0;
}

void
setConfigPath(const std::string& path)
{
	configPath = path;
}

Aggregator&
getAggregator()
{
	return aggregator;
}
