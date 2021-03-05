//
// Created by mirco on 26.02.21.
//

#ifndef UAVAP_IACTUATIONIO_H
#define UAVAP_IACTUATIONIO_H

struct ControllerOutput;

class IActuationIO
{

public:

	static constexpr const char* const typeId = "act_io";

	virtual
	~IActuationIO() = default;

	virtual void
	setControllerOutput(const ControllerOutput& out) = 0;
};

#endif //UAVAP_IACTUATIONIO_H
