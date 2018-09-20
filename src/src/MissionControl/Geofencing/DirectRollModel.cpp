/*
 * DirectRollModel.cpp
 *
 *  Created on: Aug 21, 2018
 *      Author: simonyu
 */

#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/MissionControl/Geofencing/DirectRollModel.h"

DirectRollModel::DirectRollModel() :
		rollMax_(0), g_(9.81), radiusOrbit_(0)
{
}

std::shared_ptr<DirectRollModel>
DirectRollModel::create(const boost::property_tree::ptree& config)
{
	auto directRollModel = std::make_shared<DirectRollModel>();

	if (!directRollModel->configure(config))
	{
		APLOG_ERROR << "DirectRollModel: Failed to Load Config.";
	}

	return directRollModel;
}

bool
DirectRollModel::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);

	pm.add<double>("roll_max", rollMax_, true);
	pm.add<double>("g", g_, false);

	degToRadRef(rollMax_);

	return pm.map();
}

void
DirectRollModel::notifyAggregationOnUpdate(const Aggregator& agg)
{
}

bool
DirectRollModel::updateModel(const SensorData& data)
{
	std::unique_lock<std::mutex> lock(queryMutex_, std::try_to_lock);

	if (!lock.owns_lock())
	{
		APLOG_DEBUG << "Data in use. Will update next time.";
		return false;
	}

	VehicleOneFrame frame(data.attitude[2]);

	double velocity = data.airSpeed;
	double curvature = tan(rollMax_) * g_ / velocity;

	radiusOrbit_ = velocity / curvature;
	centerOrbitLeft_ = frame.toInertialFramePosition(Vector3(-radiusOrbit_, 0, 0));
	centerOrbitRight_ = frame.toInertialFramePosition(Vector3(radiusOrbit_, 0, 0));

	return true;
}

std::vector<Vector3>
DirectRollModel::getCriticalPoints(const Edge& edge, RollDirection dir)
{
	std::unique_lock<std::mutex> lock(queryMutex_);

	std::vector<Vector3> result;
	Vector3 normal(-edge.normal.x(), -edge.normal.y(), 0);

	if (dir == RollDirection::LEFT)
	{
		result.push_back(centerOrbitLeft_ + normal * radiusOrbit_);
	}
	else
	{
		result.push_back(centerOrbitRight_ + normal * radiusOrbit_);
	}

	return result;
}
