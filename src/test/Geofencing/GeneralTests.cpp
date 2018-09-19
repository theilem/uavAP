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
 * GeneralTests.cpp
 *
 *  Created on: Aug 20, 2018
 *      Author: mircot
 */




#include <boost/test/unit_test.hpp>
#include <uavAP/Core/SensorData.h>
#include <uavAP/MissionControl/Geofencing/ConstRollRateModel.h>

BOOST_AUTO_TEST_SUITE(GeofencingTests)

BOOST_AUTO_TEST_CASE(ConstRollRateTest001_InitialConditions)
{

	ConstRollRateModel model;

	boost::property_tree::ptree config;
	config.add("roll_rate", 15);
	config.add("roll_max", 45);
	config.add("precision", 8);

	BOOST_REQUIRE(model.configure(config));


	SensorData data;
	data.position = Vector3(111,222,333);
	data.attitude = degToRad(Vector3(30, 0, 70));
	data.airSpeed = 20.0;

	model.updateModel(data);

	//Initial condition
	BOOST_CHECK_CLOSE(model.calculateYaw(30 * M_PI / 180.0, ConstRollRateModel::RollDirection::LEFT), 70 * M_PI / 180.0, 1e-8);
	BOOST_CHECK_CLOSE(model.calculateYaw(30 * M_PI / 180.0, ConstRollRateModel::RollDirection::RIGHT), 70 * M_PI / 180.0, 1e-8);

	//Yaw symmetry
	BOOST_CHECK_CLOSE(model.calculateYaw(-30 * M_PI / 180.0, ConstRollRateModel::RollDirection::LEFT), 70 * M_PI / 180.0, 1e-8);
	BOOST_CHECK_CLOSE(model.calculateYaw(-30 * M_PI / 180.0, ConstRollRateModel::RollDirection::RIGHT), 70 * M_PI / 180.0, 1e-8);

	//Initial position condition
	Vector3 pointTest1 = model.calculatePoint(30 * M_PI / 180.0, ConstRollRateModel::RollDirection::RIGHT);
	Vector3 pointTest2 = model.calculatePoint(30 * M_PI / 180.0, ConstRollRateModel::RollDirection::LEFT);
	BOOST_CHECK_SMALL(pointTest1.x() - 111, 1e-8);
	BOOST_CHECK_SMALL(pointTest1.y() - 222, 1e-8);
	BOOST_CHECK_SMALL(pointTest1.z() - 333, 1e-8);
	BOOST_CHECK_SMALL(pointTest2.x() - 111, 1e-8);
	BOOST_CHECK_SMALL(pointTest2.y() - 222, 1e-8);
	BOOST_CHECK_SMALL(pointTest2.z() - 333, 1e-8);


	//Same test with negative roll
	data.position = Vector3(111,222,333);
	data.attitude = degToRad(Vector3(-30, 0, 70));
	data.airSpeed = 20.0;

	model.updateModel(data);

	//Initial condition
	BOOST_CHECK_CLOSE(model.calculateYaw(30 * M_PI / 180.0, ConstRollRateModel::RollDirection::LEFT), 70 * M_PI / 180.0, 1e-8);
	BOOST_CHECK_CLOSE(model.calculateYaw(30 * M_PI / 180.0, ConstRollRateModel::RollDirection::RIGHT), 70 * M_PI / 180.0, 1e-8);

	//Yaw symmetry
	BOOST_CHECK_CLOSE(model.calculateYaw(-30 * M_PI / 180.0, ConstRollRateModel::RollDirection::LEFT), 70 * M_PI / 180.0, 1e-8);
	BOOST_CHECK_CLOSE(model.calculateYaw(-30 * M_PI / 180.0, ConstRollRateModel::RollDirection::RIGHT), 70 * M_PI / 180.0, 1e-8);

	//Initial position condition
	pointTest1 = model.calculatePoint(-30 * M_PI / 180.0, ConstRollRateModel::RollDirection::RIGHT);
	pointTest2 = model.calculatePoint(-30 * M_PI / 180.0, ConstRollRateModel::RollDirection::LEFT);
	BOOST_CHECK_SMALL(pointTest1.x() - 111, 1e-8);
	BOOST_CHECK_SMALL(pointTest1.y() - 222, 1e-8);
	BOOST_CHECK_SMALL(pointTest1.z() - 333, 1e-8);
	BOOST_CHECK_SMALL(pointTest2.x() - 111, 1e-8);
	BOOST_CHECK_SMALL(pointTest2.y() - 222, 1e-8);
	BOOST_CHECK_SMALL(pointTest2.z() - 333, 1e-8);

}

BOOST_AUTO_TEST_CASE(ConstRollRateTest002_CenterSymmetry_Left)
{

	ConstRollRateModel model;

	boost::property_tree::ptree config;
	config.add("roll_rate", 15);
	config.add("roll_max", 45);
	config.add("precision", 16);

	BOOST_REQUIRE(model.configure(config));

	SensorData data;
	Vector3 pos1(100,200,300);
	Vector3 att1(40, 0, 70);

	degToRadRef(att1);

	data.position = pos1;
	data.attitude = att1;
	data.airSpeed = 20.0;

	model.updateModel(data);

	Vector3 pos2 = model.calculatePoint(0, ConstRollRateModel::RollDirection::LEFT);
	Vector3 att2(0,0,0);
	att2[2] = model.calculateYaw(0, ConstRollRateModel::RollDirection::LEFT);

	data.position = pos2;
	data.attitude = att2;

	model.updateModel(data);

	Vector3 att3(40,0,0);
	degToRadRef(att3);

	Vector3 pos3 = model.calculatePoint(att3[0], ConstRollRateModel::RollDirection::LEFT);
	att3[2] = model.calculateYaw(att3[0], ConstRollRateModel::RollDirection::LEFT);

	BOOST_CHECK_SMALL((pos3 - pos1).norm(), 1e-6);
	BOOST_CHECK_SMALL((att3 - att1).norm(), 1e-6);
}

BOOST_AUTO_TEST_CASE(ConstRollRateTest003_CenterSymmetry_Right)
{

	ConstRollRateModel model;

	boost::property_tree::ptree config;
	config.add("roll_rate", 15);
	config.add("roll_max", 45);
	config.add("precision", 16);

	BOOST_REQUIRE(model.configure(config));

	SensorData data;
	Vector3 pos1(100,200,300);
	Vector3 att1(40, 0, 30);

	degToRadRef(att1);

	data.position = pos1;
	data.attitude = att1;
	data.airSpeed = 20.0;

	model.updateModel(data);

	Vector3 pos2 = model.calculatePoint(0, ConstRollRateModel::RollDirection::RIGHT);
	Vector3 att2(0,0,0);
	att2[2] = model.calculateYaw(0, ConstRollRateModel::RollDirection::RIGHT);

	data.position = pos2;
	data.attitude = att2;

	model.updateModel(data);

	Vector3 att3(40,0,0);
	degToRadRef(att3);

	Vector3 pos3 = model.calculatePoint(att3[0], ConstRollRateModel::RollDirection::RIGHT);
	att3[2] = model.calculateYaw(att3[0], ConstRollRateModel::RollDirection::RIGHT);

	BOOST_CHECK_SMALL((pos3 - pos1).norm(), 1e-6);
	BOOST_CHECK_SMALL((att3 - att1).norm(), 1e-6);
}

BOOST_AUTO_TEST_SUITE_END()
