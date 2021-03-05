//
// Created by mirco on 24.02.21.
//

#ifndef UAVAP_OVERRIDABLEVALUE_H
#define UAVAP_OVERRIDABLEVALUE_H

template<typename Type>
class OverridableValue
{
public:

	OverridableValue() = default;

	explicit
	OverridableValue(const Type& val) : value_(val)
	{}

	void
	applyOverride(bool enable, Type overrideVal)
	{
		overrideEnable_ = enable;
		overridingValue_ = overrideVal;
	}

	OverridableValue<Type>&
	operator=(const Type& val)
	{
		value_ = val;
		return *this;
	}

	operator Type() const
	{
		return overrideEnable_ ? overridingValue_ : value_;
	}

	const Type&
	operator ()() const
	{
		return overrideEnable_ ? overridingValue_ : value_;
	}

private:

	Type value_{0};
	Type overridingValue_{0};
	bool overrideEnable_{false};
};

#endif //UAVAP_OVERRIDABLEVALUE_H
