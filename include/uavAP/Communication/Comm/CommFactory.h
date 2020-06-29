/**
 *  @file         CommFactory.h
 *  @author 	Mirco Theile
 *  @date      29 June 2020
 *  @brief      UAV Autopilot Communication Comm Factory Header File
 *
 *  Description
 */

#ifndef UAVAP_COMMUNICATION_COMM_COMMFACTORY_H_
#define UAVAP_COMMUNICATION_COMM_COMMFACTORY_H_

#include <cpsCore/Framework/StaticFactory.h>

#include "uavAP/Communication/Comm/IComm.h"
#include "uavAP/Communication/Comm/IDCComm/IDCComm.h"


using CommFactory = StaticFactory<IComm, false, IDCComm>;

#endif /* UAVAP_COMMUNICATION_COMM_COMMFACTORY_H_ */
