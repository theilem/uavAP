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
 * WindPlanner.h
 *
 *  Created on: Jun 13, 2017
 *      Author: mircot
 */

#ifndef WINDPLANNER_WINDPLANNER_H_
#define WINDPLANNER_WINDPLANNER_H_

#include "FlightControl/LocalPlanner/WindPlanner/Filter.h"

class WindPlanner
{
public:
	WindPlanner();
	void
	updateStoredLine(int samples);
	void
	updateStoredTurn(int samples);
	void
	updateCompareLine(int compare);
	void
	updateCompareTurn(int compare);
	void
	updateAveragedLine(int averaged);
	void
	updateAveragedTurn(int averaged);

	/*
	 * Computes instantaneous course angle based off sensor data and location history
	 */
	float
	computeCourseLine(Vector3 location, float heading);

	/*
	 * Computes instantaneous course angle based off sensor data and location history
	 * Adds angular rate of turn as offset to account for error in past locations as a
	 * result of turning
	 */
	float
	computeCourseTurn(Vector3 location, float XYspeed, float radius, int direction, float heading);

private:
	wind_data windData;
	int sizeStored;
	int sizeUsed;
	int locationIndex; //index of entry to overwrite in lastLoc
	int courseIndex; //index of entry to overwrite in courseHistory
	bool enoughLocationsLine; //if can compute course
	bool enoughLocationsTurn;
	bool enoughCoursesLine; //if can filter course
	bool enoughCoursesTurn;
	Vector3 * locationHistory;
	float *courseHistory;
	Filter filter;
	void
	resizeNumStored();
	void
	resizeNumAveraged();
	/* Takes a pointer to wind settings and sensor data sample. Adds location to history
	 * of locations.
	 */
	void
	addLocation(Vector3 location);

	/* Takes a pointer to wind settings and a course angle. Adds course angle to history
	 * of course angles.
	 */
	void
	addCourse(float course);
	/* Takes a pointer to wind settings, flag, and instantaneous heading. Averages
	 * headings based on flag and applies Butterworth filter, then returns a course
	 * angle based off course angle history or filtered instantenous heading if not
	 * enough past headings.
	 */
	float
	filterHeadingLine(float heading);

	float
	filterHeadingTurn(float heading);
};

#endif /* WINDPLANNER_WINDPLANNER_H_ */
