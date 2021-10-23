//
// Created by seedship on 8/24/21.
//

#include <boost/property_tree/json_parser.hpp>

#include "uavAP/Core/SensorData.h"
#include "uavAP/Core/Orientation/NED.h"
#include "uavAP/FlightControl/SimplexSupervisor/MultiSimplexSupervisor.h"
#include "uavAP/FlightControl/SensingActuationIO/ISensingIO.h"
#include "uavAP/FlightControl/SensingActuationIO/IActuationIO.h"
#include "uavAP/FlightControl/Controller/ControllerOutput.h"
#include <uavAP/Core/OverrideHandler/OverrideHandler.h>
#include <uavAP/Core/DataHandling/DataHandling.h>

static VectorN<9>
mapToVector(const std::unordered_map<std::string, FloatingType>& m)
{
	decltype(mapToVector(m)) ans;
	ans[0] = m.find("u")->second;
	ans[1] = m.find("w")->second;
	ans[2] = m.find("q")->second;
	ans[3] = m.find("theta_rad")->second;
	ans[4] = m.find("v")->second;
	ans[5] = m.find("p")->second;
	ans[6] = m.find("r")->second;
	ans[7] = m.find("phi_rad")->second;
	ans[8] = m.find("h")->second;
	return ans;
}

MultiSimplexSupervisor::MultiSimplexSupervisor() : safetyController_(false), safetyAlt_(150), safetyCtrl_phiIdx(2),
												   safetyCtrl_thetaIdx(-1)
{

}

bool
MultiSimplexSupervisor::run(RunStage stage)
{
	switch (stage)
	{
		case RunStage::INIT:
		{
			// Initialize reachability regions
			std::cout << "\n20long\n";
			r20_long.fromConfig(params.r20_long_path());
			std::cout << "\n20lat\n";
			r20_lat.fromConfig(params.r20_lat_path());
			std::cout << "\n-20long\n";
			rmin20_long.fromConfig(params.rmin20_long_path());
			std::cout << "\n-20lat\n";
			rmin20_lat.fromConfig(params.rmin20_lat_path());


			if (auto oh = get<OverrideHandler>())
			{
				oh->registerOverride("simplex/safety_altitude", safetyAlt_);
			}

			// Temporary printing
			logfile_.open("/tmp/multisimplex_log" + std::to_string(timePointToNanoseconds(Clock::now())) + ".csv");
			logfile_ << "timestamp,safe,safetyCtrlActive,phiIdx,thetaIdx,phi1,phi2,phi3,phi4,phi5,theta1,theta3\n";
			logfile_ << std::scientific;
			logfile_.precision(10);
			break;
		}
		case RunStage::NORMAL:
		{
			if (auto dh = get<DataHandling>())
			{
				dh->subscribeOnData<bool>(Content::DISABLE_SAFETY_CONTROLLER, [this](const auto&)
				{
					safetyController_ = false;
				});
			}
			else
			{
				CPSLOG_DEBUG << "Missing DataHandling";
			}

//			std::cout << "\nSimplex Reachability sanity test\n";
//			VectorN<5> a;
//			a[0] = 0;
//			a[1] = 0;
//			a[2] = 0;
//			a[3] = 0;
//			a[4] = 0;
//			VectorN<5> b = a;
//			b[4] = 200;
//			std::cout << r20_long.reachable(a) << "\n";
//			std::cout << r20_long.reachable(b) << "\n";

//			Reachability polyhedron parse sanity test
//			std::cout << "\n";
//			std::cout << r20_long.regions[0].A;
//			std::cout << "\n";
//			std::cout << r20_long.regions[0].b;
//			std::cout << "\n";
//			std::cout << r20_long.regions[1400].A;
//			std::cout << "\n";
//			std::cout << r20_long.regions[1400].b;
//			std::cout << "\n";
		}
		default:
		{
			break;
		}
	}
	return false;
}

void
MultiSimplexSupervisor::setControllerOutput(const ControllerOutput& out)
{
	VectorN<4> controlInputAbs = {out.pitchOutput, out.throttleOutput, out.rollOutput, out.yawOutput};
	auto stateMap = calculateRawState();
	auto currentStateRaw = mapToVector(stateMap);
	auto Ad = this->params.a_d().calculateGainsNearest(stateMap);
	auto Bd = this->params.b_d().calculateGainsNearest(stateMap);
	auto ss = this->params.ss().calculateGainsNearest(stateMap);
	auto trim = this->params.trim().calculateGainsNearest(stateMap);
	VectorN<4> controlInputRel = controlInputAbs - trim;
	VectorN<9> currentStateRel = mapToVector(stateMap) - ss;
//	std::cout << currentStateRel << "\n";
	auto nextStateAbs = Ad * currentStateRel + Bd * controlInputRel + ss;
	std::cout << nextStateAbs << "\n";

	constexpr int p_idx[] = {2, 1, 3, 0, 4};

	bool safe = false;

	for (const auto& idx : p_idx)
	{
		auto x_hat = nextStateAbs - this->params.ss().regions()[idx].k();
		if (x_hat.transpose() * this->params.p().regions()[idx].k() * x_hat <= 1)
		{
			safe = true;
			safetyCtrl_phiIdx = idx;
			safetyCtrl_thetaIdx = -1;
			break;
		}
	}

	if (!safe)
	{
		auto x_hat_min20 = nextStateAbs - this->params.ssp().regions()[0].k();
		VectorN<5> x_longmin20;
		x_longmin20[0] = x_hat_min20[0];
		x_longmin20[1] = x_hat_min20[1];
		x_longmin20[2] = x_hat_min20[2];
		x_longmin20[3] = x_hat_min20[3];
		x_longmin20[4] = x_hat_min20[8];
		x_longmin20 += (this->params.cp_long().regions()[0].k()) / 100;
		VectorN<4> x_latmin20 = {x_hat_min20[4], x_hat_min20[5], x_hat_min20[6], x_hat_min20[7]};


		auto x_hat_20 = nextStateAbs - this->params.ssp().regions()[2].k();
		VectorN<5> x_long20;
		x_long20[0] = x_hat_20[0];
		x_long20[1] = x_hat_20[1];
		x_long20[2] = x_hat_20[2];
		x_long20[3] = x_hat_20[3];
		x_long20[4] = x_hat_20[8];
		x_long20 += (this->params.cp_long().regions()[0].k()) / 100;
		VectorN<4> x_lat20 = {x_hat_20[4], x_hat_20[5], x_hat_20[6], x_hat_20[7]};

		if (rmin20_long.reachable(x_longmin20) && rmin20_lat.reachable(x_latmin20))
		{
			safe = true;
			safetyCtrl_phiIdx = -1;
			safetyCtrl_thetaIdx = 0;
		}
		else if (r20_long.reachable(x_long20) && r20_lat.reachable(x_lat20))
		{
			safe = true;
			safetyCtrl_phiIdx = -1;
			safetyCtrl_thetaIdx = 2;
		}
	}

	if (!safe)
	{
		safetyController_ = true;
	}

	std::cout<< "Safe: " << safe << "\n";
	std::cout<< "Safety Control Active: " << safetyController_ << "\n";
	std::cout<< "PhiIdx: " << safetyCtrl_phiIdx << "\n";
	std::cout<< "ThetaIdx: " << safetyCtrl_thetaIdx << "\n";

	if (safetyController_)
	{
		ControllerOutput safeout;
		if (safetyCtrl_phiIdx != -1)
		{
			if (safetyCtrl_phiIdx < 0 || safetyCtrl_phiIdx > 4) {
				CPSLOG_ERROR << "Unexpected value for phi idx: " << safetyCtrl_phiIdx;
			}
			auto x_rel = currentStateRaw - this->params.ss().regions()[safetyCtrl_phiIdx].k();
			VectorN<4> safetyCtrl = -this->params.k().regions()[safetyCtrl_phiIdx].k() * x_rel + params.trim().regions()[safetyCtrl_phiIdx].k();
			safeout.pitchOutput = safetyCtrl[0];
			safeout.throttleOutput = safetyCtrl[1];
			safeout.rollOutput = safetyCtrl[2];
			safeout.yawOutput = safetyCtrl[3];
		}
		else
		{
			if (safetyCtrl_thetaIdx != 0 && safetyCtrl_thetaIdx != 2) {
				CPSLOG_ERROR << "Unexpected value for theta idx: " << safetyCtrl_thetaIdx;
			}
			auto x_rel = currentStateRaw - this->params.ss().regions()[safetyCtrl_thetaIdx].k();
			VectorN<5> x_long;
			x_long[0] = x_rel[0];
			x_long[1] = x_rel[1];
			x_long[2] = x_rel[2];
			x_long[3] = x_rel[3];
			x_long[4] = x_rel[8];
			VectorN<4> x_lat = {x_rel[4], x_rel[5], x_rel[6], x_rel[7]};

			Vector2 out_long = -this->params.kp_long().regions()[safetyCtrl_phiIdx].k() * x_long;
			Vector2 out_lat = -this->params.kp_lat().regions()[safetyCtrl_phiIdx].k() * x_lat;
			VectorN<4> safetyCtrl = {out_long[0], out_long[1], out_lat[0], out_lat[1]};
			safetyCtrl += params.trimp().regions()[safetyCtrl_thetaIdx].k();
		}
		safeout.pitchOutput = std::clamp<FloatingType>(safeout.pitchOutput, -1, 1);
		safeout.throttleOutput = std::clamp<FloatingType>(safeout.throttleOutput, -1, 1);
		safeout.rollOutput = std::clamp<FloatingType>(safeout.rollOutput, -1, 1);
		safeout.yawOutput = std::clamp<FloatingType>(safeout.yawOutput, -1, 1);
		if (auto io = get<IActuationIO>())
		{
			io->setControllerOutput(safeout);
			return;
		}
		CPSLOG_ERROR << "Missing Actuation IO";
	}

	if (auto io = get<IActuationIO>())
	{
		io->setControllerOutput(out);
		return;
	}
	CPSLOG_ERROR << "Missing Actuation IO";
}

std::unordered_map<std::string, FloatingType>
MultiSimplexSupervisor::calculateRawState() const
{
	decltype(calculateRawState()) ans;
	if (auto io = get<ISensingIO>())
	{
		SensorData sd = io->getSensorData();
		NED::convert(sd, Frame::BODY);
		ans["u"] = sd.velocity.x();
		ans["w"] = sd.velocity.z();
		ans["q"] = sd.angularRate.y();
		ans["theta_rad"] = sd.attitude.y();
		ans["v"] = sd.velocity.y();
		ans["p"] = sd.angularRate.x();
		ans["r"] = sd.angularRate.z();
		ans["phi_rad"] = sd.attitude.x();
		ans["h"] = -safetyAlt_ - sd.position.z();
	}
	else
	{
		CPSLOG_ERROR << "Missing Sensing IO!";
	}
	return ans;
}

template<std::size_t dim>
void
MultiSimplexSupervisor::initializeReachabilitySet(ReachabilitySet<dim>& set, const std::string& path)
{
	Configuration config;
	boost::property_tree::read_json(path, config);
	set.fromConfig(path);
}
