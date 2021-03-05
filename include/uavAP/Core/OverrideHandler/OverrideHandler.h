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

	void
	registerOverride(const std::string& overrideId, const OverrideHandle& overrideHandle);

	void
	registerOverride(const std::string& overrideId, OverridableValue<FloatingType>& overridableValue);

	void
	registerOverride(const std::string& overrideId, OverridableValue<Angle<FloatingType>>& overridableValue);

	bool
	applyOverride(const std::string& overrideId, bool enable, FloatingType value);

	void
	applyOverrides(const std::map<std::string, FloatingType>& overrides);

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
	bool enabled_{true};
};

#endif //UAVAP_OVERRIDEHANDLER_H
