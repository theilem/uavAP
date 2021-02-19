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
 * FramesTest.cpp
 *
 *  Created on: Aug 8, 2018
 *      Author: mircot
 */

#include <cpsCore/Utilities/Test/TestInfo.h>
#include <uavAP/Core/Frames/BodyFrame.h>
#include <uavAP/Core/Frames/InertialFrame.h>

TEST_CASE("Body Frame Test 1")
{
	Vector3 bodyRot(90, 0, 0);
	BodyFrame body(bodyRot * M_PI / 180.0);

	InertialFrame inert;

	Vector3 dirBody(1, 0, 0);

	CHECK(inert.fromFrameDirection(body, dirBody) == body.toInertialFrameDirection(dirBody));
	CHECK(inert.fromFrameDirection(body, dirBody) == dirBody);


	dirBody = Vector3(0, 1, 0);

	CHECK(inert.fromFrameDirection(body, dirBody) == body.toInertialFrameDirection(dirBody));
	CHECK(inert.fromFrameDirection(body, dirBody).x() == Approx(0).margin(1e-6));
	CHECK(inert.fromFrameDirection(body, dirBody).y() == Approx(0).margin(1e-6));
	CHECK(inert.fromFrameDirection(body, dirBody).z() == Approx(1).margin(1e-6));

}

TEST_CASE("Body Frame Test 2")
{
	Vector3 bodyRot(45, 30, 90);
	BodyFrame body(bodyRot * M_PI / 180.0);

	InertialFrame inert;

	Vector3 dirBody(1, 0, 0);

	CHECK(inert.fromFrameDirection(body, dirBody) == body.toInertialFrameDirection(dirBody));
	CHECK(inert.fromFrameDirection(body, dirBody) == dirBody);


	dirBody = Vector3(0, 1, 0);

	CHECK(inert.fromFrameDirection(body, dirBody) == body.toInertialFrameDirection(dirBody));
	CHECK(inert.fromFrameDirection(body, dirBody).x()  == Approx(0).margin(1e-6));
	CHECK(inert.fromFrameDirection(body, dirBody).y() == Approx(0).margin(1e-6));
	CHECK(inert.fromFrameDirection(body, dirBody).z() == Approx(1).margin(1e-6));

}
