//
// Created by seedship on 8/24/21.
//

#ifndef UAVAP_MULTISIMPLEXSUPERVISOR_H
#define UAVAP_MULTISIMPLEXSUPERVISOR_H

#include <cpsCore/cps_object>
#include "uavAP/FlightControl/SimplexSupervisor/ISimplexSupervisor.h"

#include "uavAP/FlightControl/StateSpaceUtils/GainScheduler.h"
#include "uavAP/FlightControl/StateSpaceUtils/ReachabilityRegion.h"

#include "uavAP/Core/OverrideHandler/OverridableValue.hpp"

#include <cpsCore/Configuration/Parameter.hpp>

struct MultiSimplexParams
{
	// Length 7, first 5 equilibrium states second 2 nonequilibrium
	Parameter<GainSchedulingParams<9, 9>> a_d = {{}, "Ad", true};
	Parameter<GainSchedulingParams<9, 4>> b_d = {{}, "Bd", true};
	Parameter<GainSchedulingParams<9>> c = {{}, "C", true};
	Parameter<GainSchedulingParams<9>> ss = {{}, "ss", true};
	Parameter<GainSchedulingParams<4>> trim = {{}, "trim", true};

	// Length 5
	Parameter<GainSchedulingParams<9, 9>> p = {{}, "p", true};
	Parameter<GainSchedulingParams<4, 9>> k = {{}, "k", true};

	//Length 2
	Parameter<GainSchedulingParams<2, 5>> kp_long = {{}, "kp_long", true};
	Parameter<GainSchedulingParams<2, 4>> kp_lat = {{}, "kp_lat", true};
	Parameter<GainSchedulingParams<9>> ssp = {{}, "ssp", true};
	Parameter<GainSchedulingParams<4>> trimp = {{}, "trimp", true};
	Parameter<GainSchedulingParams<5>> cp_long = {{}, "cp_long", true};

	Parameter<std::string> r20_long_path = {"", "r20_long_path", true};
	Parameter<std::string> r20_lat_path = {"", "r20_lat_path", true};

	Parameter<std::string> rmin20_long_path = {"", "rmin20_long_path", true};
	Parameter<std::string> rmin20_lat_path = {"", "rmin20_lat_path", true};

	template<typename Config>
	inline void
	configure(Config& c)
	{
		c & a_d;
		c & b_d;
		c & ss;
		c & this->c;
		c & trim;

		c & p;
		c & k;

		c & kp_long;
		c & kp_lat;

		c & ssp;
		c & trimp;

		c & cp_long;

		c & r20_long_path;
		c & r20_lat_path;
		c & rmin20_long_path;
		c & rmin20_lat_path;
	}
};

class IActuationIO;

class ISensingIO;

class DataHandling;

class OverrideHandler;

class MultiSimplexSupervisor: public AggregatableObject<IActuationIO, ISensingIO, DataHandling, OverrideHandler>,
							  public ConfigurableObject<MultiSimplexParams>,
							  public IRunnableObject,
							  public ISimplexSupervisor
{
public:
	static constexpr TypeId typeId = "multi_simplex";

	MultiSimplexSupervisor();

	bool
	run(RunStage stage) override;

	void
	setControllerOutput(const ControllerOutput& out) override;

private:
	std::unordered_map<std::string, FloatingType>
	calculateRawState() const;

	template<std::size_t dim>
	void
	initializeReachabilitySet(ReachabilitySet<dim>& set, const std::string& path);

	bool safetyController_;

	ReachabilitySet<5> r20_long;
	ReachabilitySet<4> r20_lat;
	ReachabilitySet<5> rmin20_long;
	ReachabilitySet<4> rmin20_lat;

	OverridableValue<FloatingType> safetyAlt_;

	int safetyCtrl_phiIdx;
	int safetyCtrl_thetaIdx;
};


#endif //UAVAP_MULTISIMPLEXSUPERVISOR_H
