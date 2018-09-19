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
 * SplineGlobalPlanner.cpp
 *
 *  Created on: Dec 16, 2017
 *      Author: mircot
 */
#include "uavAP/MissionControl/GlobalPlanner/PathSections/CubicSpline.h"
#include "uavAP/MissionControl/GlobalPlanner/PathSections/Orbit.h"
#include "uavAP/MissionControl/GlobalPlanner/Trajectory.h"
#include "uavAP/MissionControl/GlobalPlanner/SplineGlobalPlanner/SplineGlobalPlanner.h"
#include "uavAP/Core/IPC/IPC.h"
#include "uavAP/Core/PropertyMapper/PropertyMapper.h"
#include "uavAP/Core/DataPresentation/BinarySerialization.hpp"
#include <Eigen/Core>

SplineGlobalPlanner::SplineGlobalPlanner() :
		tau_(0.5), inclusionLength_(0), smoothenZ_(true), naturalSplines_(false)
{
}

std::shared_ptr<IGlobalPlanner>
SplineGlobalPlanner::create(const boost::property_tree::ptree& config)
{
	auto gp = std::make_shared<SplineGlobalPlanner>();
	gp->configure(config);
	return gp;
}

bool
SplineGlobalPlanner::configure(const boost::property_tree::ptree& config)
{
	PropertyMapper pm(config);
	pm.add<double>("tau", tau_, false);
	pm.add<bool>("smoothen_z", smoothenZ_, false);
	pm.add<bool>("natural", naturalSplines_, false);
	pm.add<uint8_t>("inclusion_length", inclusionLength_, false);
	return pm.map();
}

bool
SplineGlobalPlanner::run(RunStage stage)
{
	switch (stage)
	{
	case RunStage::INIT:
	{
		if (!ipc_.isSet())
		{
			APLOG_ERROR << "GlobalPlanner: IPC missing.";
			return true;
		}
		auto ipc = ipc_.get();
		trajectoryPublisher_ = ipc->publishPackets("trajectory");
		break;
	}
	case RunStage::NORMAL:
	{
		break;
	}
	case RunStage::FINAL:
		break;
	default:
		break;
	}
	return false;
}

void
SplineGlobalPlanner::notifyAggregationOnUpdate(const Aggregator& agg)
{
	ipc_.setFromAggregationIfNotSet(agg);
}

void
SplineGlobalPlanner::setMission(const Mission& mission)
{
	if (mission.waypoints.empty())
	{
		APLOG_ERROR << "Mission does not contain Waypoints. Ignore.";
		return;
	}

	mission_ = mission;

	bool infinite = mission.infinite;

	Trajectory traj;

	if (infinite && naturalSplines_)
	{
		traj = createNaturalSplines(mission);
	}
	else
	{
		traj = createCatmulRomSplines(mission);
	}

	APLOG_DEBUG << "Send Trajectory";
	auto packet = dp::serialize(traj);
	trajectoryPublisher_.publish(packet);
}

Mission
SplineGlobalPlanner::getMission() const
{
	return mission_;
}

Trajectory
SplineGlobalPlanner::createNaturalSplines(const Mission& mission)
{
	const auto& wp = mission.waypoints;
	uint8_t n = inclusionLength_ * 2;
	if (n == 0)
		n = static_cast<uint8_t>(wp.size());

	Eigen::Matrix3d A, B, K, L, Rn;
	A << 1, 1, 1, 1, 2, 3, 0, 1, 3;

	B << 0, 0, 0, -1, 0, 0, 0, -1, 0;

	K << 3, -2, 1, -1, 3, -2, 1, -1, 1;

	L << -2, 1, 0, 3, -2, 0, -1, 1, 0;

	auto Lpow = L;
	//TODO Fix stupidness
	for (uint8_t i = 0; i < n - 1; ++i)
	{
		Lpow = Lpow * L;
	}

	Rn = (A + B * Lpow).inverse();

	std::cout << "Rn: " << Rn << std::endl;

	std::vector<Eigen::Matrix3d> P;

	for (uint8_t j = 0; j < n; ++j)
	{
		Eigen::Matrix3d temp;
		temp.setZero();
		if (j == n - 1)
			temp.row(0) = wp[0].location - wp[j].location;
		else
			temp.row(0) = wp[j + 1].location - wp[j].location;

		P.push_back(Rn * temp);
	}

	auto C = P;
	for (size_t i = 1; i < n; ++i)
	{
		for (size_t j = 0; j < n; ++j)
		{
			P[j] = L * P[j];
			auto k = (j + i) % (n - 1);
			C[k] += P[j];
			std::cout << C[j] << std::endl << std::endl;
		}
		std::cout << "Next" << std::endl << std::endl;
	}

	PathSections traj;
	for (size_t j = 0; j < n; ++j)
	{
		double velocity = (!wp[j].velocity) ? mission.velocity : *wp[j].velocity;

		auto spline = std::make_shared<CubicSpline>(wp[j].location, C[j].row(0), C[j].row(1),
				C[j].row(2), velocity);
		traj.push_back(spline);

		std::cout << spline->c0_ << ", " << spline->c1_ << ", " << spline->c2_ << ", "
				<< spline->c3_ << ", " << std::endl;
	}

	return Trajectory(traj, mission.infinite);

}

Trajectory
SplineGlobalPlanner::createCatmulRomSplines(const Mission& mission)
{

	const auto& wp = mission.waypoints;
	bool infinite = mission.infinite;

	PathSections traj;

	Eigen::Matrix<double, 4, 4> tauMat;
	tauMat << 0, 1, 0, 0, -tau_, 0, tau_, 0, 2 * tau_, tau_ - 3, 3 - 2 * tau_, -tau_, -tau_, 2
			- tau_, tau_ - 2, tau_;
	Eigen::Matrix<double, 4, 3> pointMat;
	Eigen::Matrix<double, 4, 3> approachMat;
	if (wp.size() == 1)
		infinite = false;
	else
	{
		if (infinite)
			pointMat.row(0) = wp.back().location.transpose();
		else
			pointMat.row(0) = wp.front().location.transpose();
		pointMat.row(1) = wp.front().location.transpose();
		pointMat.row(2) = wp[1].location.transpose();
	}

	bool populateApproach = mission.initialPosition.is_initialized();

	for (auto it = wp.begin(); it != wp.end(); ++it)
	{
		if (it->direction)
			pointMat.row(0) = pointMat.row(2) - it->direction->transpose();
		else
		{
			if (smoothenZ_)
			{
				const auto& col = pointMat.col(2);
				double incline1 = col[2] - col[0];
				double incline2 = col[2] - col[1];
				double incline3 = col[1] - col[0];
				double minIncline = 0;
				if (fabs(incline1) < fabs(incline2))
				{
					if (fabs(incline3) < fabs(incline1))
						minIncline = incline3;
					else
						minIncline = incline1;
				}
				else
				{
					if (fabs(incline3) < fabs(incline2))
						minIncline = incline3;
					else
						minIncline = incline2;
				}

				pointMat.col(2)[0] = col[2] - minIncline;
			}
		}

		if (populateApproach)
		{
			approachMat.bottomRows(3) = pointMat.topRows(3);
			populateApproach = false;
		}

		auto nextIt = it + 1;
		if (nextIt == wp.end())
		{
			if (!infinite)
			{
				double velocity = (!it->velocity) ? mission.velocity : *it->velocity;
				traj.push_back(
						std::make_shared<Orbit>(it->location, Vector3::UnitZ(), 50, velocity));
				break;
			}
			nextIt = wp.begin();
		}
		auto next = nextIt + 1;
		if (next == wp.end())
		{
			if (!infinite)
				next = nextIt;
			else
				next = wp.begin();
		}

		if (nextIt->direction)
			pointMat.row(3) = pointMat.row(1) + nextIt->direction->transpose();
		else
		{
			pointMat.row(3) = next->location;
			if (smoothenZ_)
			{
				const auto& col = pointMat.col(2);
				double incline1 = col[3] - col[1];
				double incline2 = col[3] - col[2];
				double incline3 = col[2] - col[1];
				double minIncline = 0;
				if (fabs(incline1) < fabs(incline2))
				{
					if (fabs(incline3) < fabs(incline1))
						minIncline = incline3;
					else
						minIncline = incline1;
				}
				else
				{
					if (fabs(incline3) < fabs(incline2))
						minIncline = incline3;
					else
						minIncline = incline2;
				}

				pointMat.col(2)[3] = col[1] + minIncline;
			}
		}

		Eigen::Matrix<double, 3, 4> C = (tauMat * pointMat).transpose();
		double velocity = (!it->velocity) ? mission.velocity : *it->velocity;
		auto spline = std::make_shared<CubicSpline>(C.col(0), C.col(1), C.col(2), C.col(3),
				velocity);
		traj.push_back(spline);

		pointMat.topRows(3) = pointMat.bottomRows(3);

		pointMat.row(2) = next->location;
	}

	Trajectory result(traj, mission.infinite);

	if (mission.initialPosition)
	{
		Waypoint init = *mission.initialPosition;
		approachMat.row(1) = init.location.transpose();
		if (init.direction)
			approachMat.row(0) = approachMat.row(2) - (init.direction->transpose() * mission.velocity);
		else
			approachMat.row(0) = approachMat.row(1);

		Eigen::Matrix<double, 3, 4> C = (tauMat * approachMat).transpose();
		auto spline = std::make_shared<CubicSpline>(C.col(0), C.col(1), C.col(2), C.col(3),
				mission.velocity);
		result.approachSection = spline;
	}

	return result;
}
