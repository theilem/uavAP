//
// Created by seedship on 8/24/21.
//

#include "uavAP/Core/SensorData.h"
#include "uavAP/Core/Orientation/NED.h"
#include "uavAP/FlightControl/SimplexSupervisor/MultiSimplexSupervisor.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"
#include "uavAP/FlightControl/SensingActuationIO/IActuationIO.h"

void
MultiSimplexSupervisor::setControllerOutput(const ControllerOutput& out)
{

}

VectorN<9>
MultiSimplexSupervisor::calculateRawState() const
{
	VectorN<9> ans;
	if (auto io = get<ISensingIO>())
	{
		SensorData sd = io->getSensorData();
		NED::convert(sd, Frame::BODY);
		ans[0] = sd.velocity.x();
		ans[1] = sd.velocity.z();
		ans[2] = sd.angularRate.y();
		ans[3] = sd.attitude.y();
		ans[4] = sd.velocity.y();
		ans[5] = sd.angularRate.x();
		ans[6] = sd.angularRate.z();
		ans[7] = sd.attitude.x();
		ans[8] = -sd.position.z();
	}
	else
	{
		CPSLOG_ERROR << "Missiong Sensing IO!";
	}
	return ans;
}
