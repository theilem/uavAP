/*
 * IWindEstimation.h
 *
 *  Created on: Feb 8, 2019
 *      Author: mirco
 */

#ifndef UAVAP_FLIGHTANALYSIS_WINDESTIMATION_IWINDESTIMATION_H_
#define UAVAP_FLIGHTANALYSIS_WINDESTIMATION_IWINDESTIMATION_H_
#include <uavAP/FlightAnalysis/WindEstimation/WindInfo.h>


class IWindEstimation
{
public:

	virtual ~IWindEstimation() = default;

	virtual WindInfo
	getWindInfo() const = 0;

};


#endif /* UAVAP_FLIGHTANALYSIS_WINDESTIMATION_IWINDESTIMATION_H_ */
