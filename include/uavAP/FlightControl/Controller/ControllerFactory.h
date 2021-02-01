/*
 * FlightControllerFactory.h
 *
 *  Created on: Jun 5, 2017
 *      Author: mircot
 */

#ifndef FLIGHTCONTROLLER_FLIGHTCONTROLLERFACTORY_H_
#define FLIGHTCONTROLLER_FLIGHTCONTROLLERFACTORY_H_

#include <cpsCore/Framework/StaticFactory.h>
#include "uavAP/FlightControl/Controller/PIDController/RatePIDController/RatePIDController.h"
#include "uavAP/FlightControl/Controller/PIDController/ManeuverRatePIDController/ManeuverRatePIDController.h"
#include "uavAP/FlightControl/Controller/StateSpaceController/PitchStateSpaceController.h"

using ControllerFactory = StaticFactory<IController, false,
		RatePIDController,
		ManeuverRatePIDController,
		PitchStateSpaceController>;

#endif /* FLIGHTCONTROLLER_FLIGHTCONTROLLERFACTORY_H_ */
