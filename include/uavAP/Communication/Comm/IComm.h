/**
 *  @file         IComm.h
 *  @author Simon Yu
 *  @date      31 July 2017
 *  @brief      UAV Autopilot Communication Comm Interface Header File
 *
 *  Description
 */

#ifndef UAVAP_COMMUNICATION_COMM_ICOMM_H_
#define UAVAP_COMMUNICATION_COMM_ICOMM_H_

class IComm
{
public:

	static constexpr const char* const typeId = "comm";

	virtual
	~IComm() = default;
};

#endif /* UAVAP_COMMUNICATION_COMM_ICOMM_H_ */
