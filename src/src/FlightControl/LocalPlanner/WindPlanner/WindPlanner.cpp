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
 * WindPlanner.cpp
 *
 *  Created on: Jun 13, 2017
 *      Author: uav
 */

#include <FlightControl/LocalPlanner/WindPlanner/WindPlanner.h"
#include "uavAP/Communication/constants.h"
WindPlanner::WindPlanner() :
		sizeStored(0), sizeUsed(0), locationIndex(0), courseIndex(0), enoughLocationsLine(false), enoughLocationsTurn(
				false), enoughCoursesLine(false), enoughCoursesTurn(false), locationHistory(
				nullptr), courseHistory(nullptr)
{
}

void
WindPlanner::updateStoredLine(int samples)
{
	windData.samplesStoredLine = samples;
	resizeNumStored();
}

void
WindPlanner::updateStoredTurn(int samples)
{
	windData.samplesStoredTurn = samples;
	resizeNumStored();
}

void
WindPlanner::resizeNumStored()
{
	sizeStored = std::max(windData.samplesStoredLine, windData.samplesStoredTurn);
	locationHistory = (Vector3*) realloc(locationHistory, sizeStored * sizeof(Vector3));
	locationIndex = 0;
	enoughLocationsLine = false;
	enoughLocationsTurn = false;
	printf("resizing sizeStored to %i
", sizeStored);
}

void
WindPlanner::updateCompareLine(int compare)
{
	windData.locCompareLine = compare;
}

void
WindPlanner::updateCompareTurn(int compare)
{
	windData.locCompareTurn = compare;
}

void
WindPlanner::updateAveragedLine(int averaged)
{
	windData.avgsUsedLine = averaged;
	resizeNumAveraged();
}

void
WindPlanner::updateAveragedTurn(int averaged)
{
	windData.avgsUsedTurn = averaged;
	resizeNumAveraged();
}

void
WindPlanner::resizeNumAveraged()
{
	sizeUsed = std::max(windData.avgsUsedLine, windData.avgsUsedTurn);
	courseHistory = (float*) realloc(courseHistory, sizeUsed * sizeof(float));
	courseIndex = 0;
	enoughCoursesLine = false;
	enoughCoursesTurn = false;
	printf("resizing sizeUsed to %i
", sizeUsed);
}

float
WindPlanner::computeCourseLine(Vector3 location, float heading)
{
	addLocation(location);
	if (enoughLocationsLine)
	{
		//data->locHistory pointer to index to overwrite
		//data->windData.locCompareLine number of cycles we are looking back
		//Adding sizeStored to prevent negative numbers
		float Northing, Easting, course;
		Northing = location.x()
				- locationHistory[(sizeStored + locationIndex - windData.locCompareLine)
						% sizeStored].x();
		Easting = location.y()
				- locationHistory[(sizeStored + locationIndex - windData.locCompareLine)
						% sizeStored].y();
		course = atan2(Easting, Northing);
		addCourse(course);
	}
	else
	{
		addCourse(heading);
		return heading;
	}
	return filterHeadingLine(heading);

}

float
WindPlanner::computeCourseTurn(Vector3 location, float XYspeed, float radius, int direction,
		float heading)
{
	addLocation(location);
	if (enoughLocationsTurn)
	{
		float course, Northing, Easting;
		Northing = location.x()
				- locationHistory[(sizeStored + locationIndex - windData.locCompareTurn)
						% sizeStored].x();
		Easting = location.y()
				- locationHistory[(sizeStored + locationIndex - windData.locCompareTurn)
						% sizeStored].y();
		course = atan2(Easting, Northing);
		float turnangle = XYspeed * windData.locCompareTurn * MD_PERIOD / 1000000 / radius;
		turnangle *= direction;
		course += turnangle;
		addCourse(course);
	}
	else
	{
		addCourse(heading);
		return heading;
	}
	return filterHeadingTurn(heading);
}

void
WindPlanner::addLocation(Vector3 location)
{
	if (locationIndex < sizeStored)
	{
		//data->locHistory++ to overwrite index at data->locHistory then increment it after overwriting
		locationHistory[locationIndex++] = location;
		if (locationIndex == windData.locCompareLine - 1)
			enoughLocationsLine = true;
		if (locationIndex == windData.locCompareTurn - 1)
			enoughLocationsTurn = true;
	}
	else
	{//note this function is only called when wind correction is turned on so no check for sizeStored==0
		locationIndex %= sizeStored;
		locationHistory[locationIndex++] = location;
	}
}

void
WindPlanner::addCourse(float heading)
{
	if (courseIndex < sizeUsed)
	{
		//data->avgHistory++ to overwrite index at data->avgHistory then increment it after overwriting
		courseHistory[courseIndex++] = heading;
		if (courseIndex == windData.avgsUsedLine - 1)
			enoughCoursesLine = true;
		if (courseIndex == windData.avgsUsedTurn - 1)
			enoughCoursesTurn = true;
	}
	else
	{ //XXX this function is only called when wind correction is turned on so no check for sizeUsed!=0
		courseIndex %= sizeUsed;
		courseHistory[courseIndex++] = heading;
	}
}

float
WindPlanner::filterHeadingLine(float heading)
{
	float course = 0;
	if (enoughCoursesLine)
	{
		for (int ind = 1; ind <= windData.avgsUsedLine; ind++)
		{
			//data->avgHistory pointer to index to overwrite
			//ind number of cycles we are looking back
			//Adding sizeUsed to prevent negative numbers
			course += courseHistory[(sizeUsed + courseIndex - ind) % sizeUsed];
		}
		course /= windData.avgsUsedLine;
		return filter.filter(course);
	}

	return filter.filter(heading);
}

float
WindPlanner::filterHeadingTurn(float heading)
{
	float course = 0;
	if (enoughCoursesLine)
	{
		for (int ind = 1; ind <= windData.avgsUsedTurn; ind++)
		{
			//data->avgHistory pointer to index to overwrite
			//ind number of cycles we are looking back
			//Adding sizeUsed to prevent negative numbers
			course += courseHistory[(sizeUsed + courseIndex - ind) % sizeUsed];
		}
		course /= windData.avgsUsedTurn;
		return filter.filter(course);
	}

	return filter.filter(heading);
}
