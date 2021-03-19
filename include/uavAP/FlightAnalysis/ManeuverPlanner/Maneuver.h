/*
 * Maneuver.h
 *
 *  Created on: Sep 16, 2017
 *      Author: mircot
 */

#ifndef UAVAP_MISSIONCONTROL_MISSIONPLANNER_MANEUVERPLANNER_MANEUVERSET_H_
#define UAVAP_MISSIONCONTROL_MISSIONPLANNER_MANEUVERPLANNER_MANEUVERSET_H_

#include <vector>

#include "cpsCore/cps_object"
#include "uavAP/FlightAnalysis/Condition/ICondition.h"
#include "uavAP/FlightAnalysis/SignalGenerator/ISignalGenerator.h"

//struct Maneuver
//{
//	AdvancedControl advancedControl;
//	std::shared_ptr<ICondition> condition;
//	std::map<ControllerOutputs, bool> controllerOutputOverrideMap;
//	ControllerOutputsOverrides controllerOutputOverrideType = ControllerOutputsOverrides::INVALID;
//	bool controllerOutputOverrideFlag = false;
//	bool analyzeManeuver = false;
//	bool analyzeTrim = false;
//
//	Parameter<Override> override = {{}, "override", false};
//
//	bool
//	configure(const Configuration& config)
//	{
//		PropertyMapper<Configuration> pm(config);
//		Configuration conditionTree;
//		Configuration controllerOutputOverrideTree;
//		ConditionFactory factory;
//
//		pm.add<Override>("override", override, true);
//
//		if (pm.add("override_controller_outputs", controllerOutputOverrideTree, false))
//		{
//			PropertyMapper<Configuration> controllerOutputOverridePm(controllerOutputOverrideTree);
//
//			controllerOutputOverridePm.addEnum<ControllerOutputsOverrides>("type",
//					controllerOutputOverrideType, true);
//
//			for (auto& overrideIt : controllerOutputOverrideTree)
//			{
//				if (overrideIt.first == "enable")
//				{
//					controllerOutputOverridePm.add<bool>(overrideIt.first,
//							controllerOutputOverrideFlag, true);
//				}
//				else
//				{
//					if (!controllerOutputOverrideFlag)
//					{
//						continue;
//					}
//
//					bool overrideOutput = false;
//
//					if (controllerOutputOverridePm.add<bool>(overrideIt.first, overrideOutput,
//							true))
//					{
//						auto controllerOutputsEnum = EnumMap<ControllerOutputs>::convert(
//								overrideIt.first);
//						controllerOutputOverrideMap.insert(
//								std::make_pair(controllerOutputsEnum, overrideOutput));
//					}
//				}
//			}
//		}
//
//		pm.add("condition", conditionTree, true);
//		pm.add<AdvancedControl>("advanced_control", advancedControl, false);
//		pm.add<bool>("analyze_maneuver", analyzeManeuver, false);
//		pm.add<bool>("analyze_trim", analyzeTrim, false);
//
//		condition = ConditionFactory::create(conditionTree);
//
//		return pm.map();
//	}
//};

using ManeuverOverride = std::unordered_map<std::string, FloatingType>;

struct ManeuverParams
{
	Parameter<ManeuverOverride> overrides = {{}, "overrides", true};
	Parameter<std::map<std::string, Configuration>> waveforms = {{}, "waveforms", false};
	Parameter<Configuration> transition = {{}, "transition", true};

	template<typename Config>
	void
	configure(Config& c)
	{
		c & overrides;
		c & waveforms;
		c & transition;
	}
};

class Maneuver : public ConfigurableObject<ManeuverParams>
{
public:

	bool
	initialize(const Aggregator& aggregator);

	ManeuverOverride
	getOverrides() const;

	bool
	inTransition();

	void
	printInfo();

	bool
	isTimeVarying() const;

private:

	std::shared_ptr<ICondition> transition_;
	std::map<std::string, std::shared_ptr<ISignalGenerator>> waveforms_;

};

struct ManeuverSet
{
	using iterator = std::vector<ManeuverParams>::iterator;
	using const_iterator = std::vector<ManeuverParams>::const_iterator;

	Parameter<std::vector<ManeuverParams>> maneuvers = {{}, "maneuvers", true};

	template<typename Config>
	void
	configure(Config& c)
	{
		c & maneuvers;
	}
};

/**
 * Describes the current active maneuver for the ground station
 */
struct ManeuverDescriptor
{
	std::string maneuverName;
	std::vector<ManeuverOverride> overrides;
	std::vector<std::string> conditions;

	template<typename Config>
	void
	configure(Config& c)
	{
		c & maneuverName;
		c & overrides;
		c & conditions;
	}
};

//using ManeuverSet = std::vector<Maneuver>;

#endif /* UAVAP_MISSIONCONTROL_MISSIONPLANNER_MANEUVERPLANNER_MANEUVERSET_H_ */
