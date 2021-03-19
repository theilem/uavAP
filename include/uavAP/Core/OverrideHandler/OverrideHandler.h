//
// Created by mirco on 23.02.21.
//

#ifndef UAVAP_OVERRIDEHANDLER_H
#define UAVAP_OVERRIDEHANDLER_H

#include <cpsCore/cps_object>
#include "uavAP/Core/OverrideHandler/OverridableValue.hpp"

class DataHandling;
class IPC;

class OverrideHandler: public AggregatableObject<DataHandling, IPC>, public IRunnableObject
{
public:

	static constexpr TypeId typeId = "override_handler";

	using OverrideHandle = std::function<void(bool, FloatingType)>;
	using MaintainHandle = std::function<void(bool)>;

	void
	registerOverride(const std::string& overrideId, const OverrideHandle& overrideHandle);

	void
	registerMaintain(const std::string& overrideId, const MaintainHandle& maintainHandle);

	void
	registerOverride(const std::string& overrideId, OverridableValue<FloatingType>& overridableValue);

	void
	registerOverride(const std::string& overrideId, MaintainableValue<FloatingType>& trimmableValue);

	void
	registerOverride(const std::string& overrideId, OverridableValue<Angle<FloatingType>>& overridableValue);

	void
	registerOverride(const std::string& overrideId, MaintainableValue<Angle<FloatingType>>& trimmableValue);

	bool
	applyOverride(const std::string& overrideId, bool enable, FloatingType value);

	void
	applyOverrides(const std::map<std::string, FloatingType>& overrides);

	void
	applyMaintains(const std::vector<std::string>& maintains);

	void
	disableAllOverrides();

	std::vector<std::string>
	getPossibleOverrideIds() const;

	bool
	run(RunStage stage) override;

	void
	enable();

	void
	disable();

private:

	std::map<std::string, OverrideHandle> overrideHandles_;
	std::map<std::string, MaintainHandle> maintainHandles_;
	bool enabled_{true};
};

#endif //UAVAP_OVERRIDEHANDLER_H
