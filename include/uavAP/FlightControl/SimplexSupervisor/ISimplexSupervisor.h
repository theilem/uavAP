//
// Created by seedship on 8/24/21.
//

#ifndef UAVAP_ISIMPLEXCONTROLLER_H
#define UAVAP_ISIMPLEXCONTROLLER_H

enum class RecoveryMode
{
	TIME,
	VALUE
};

struct ControllerOutput;

class ISimplexSupervisor
{
public:

	static constexpr const char* const typeId = "simplex_supervisor";

	virtual
	~ISimplexSupervisor() = default;

//	virtual void
//	turnOffSafetyController() = 0;
//
//	virtual void
//	turnOnSafetyController(long duration_ms = 0) = 0;

	virtual void
	setControllerOutput(const ControllerOutput& out) = 0;

protected:

//	RecoveryMode recoveryMode_;

};

#endif //UAVAP_ISIMPLEXCONTROLLER_H
