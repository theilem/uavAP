//
// Created by mirco on 24.02.21.
//

#ifndef UAVAP_OVERRIDABLEVALUE_H
#define UAVAP_OVERRIDABLEVALUE_H

#include <cpsCore/Utilities/Filtering.hpp>
#include <iostream>

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

	Type
	operator()() const
	{
		return overrideEnable_ ? overridingValue_ : value_;
	}

protected:

	Type value_{0};
	Type overridingValue_{0};
	bool overrideEnable_{false};
};

template<typename Type>
class MaintainableValue : public OverridableValue<Type>
{
public:

	MaintainableValue(const Type& alpha, const Type& init) : filteredValue_(alpha, init)
	{}

	void
	maintainValue(bool enable)
	{
		maintainEnabled_ = enable;
	}

	MaintainableValue<Type>&
	operator=(const Type& val)
	{
		this->value_ = val;
		filteredValue_ = this->operator()();
		return *this;
	}

	operator Type() const
	{
		return this->overrideEnable_ ? this->overridingValue_ :
			   this->maintainEnabled_ ? this->filteredValue_
									  : this->value_;
	}

	Type
	operator()() const
	{
		return this->overrideEnable_ ? this->overridingValue_ :
			   this->maintainEnabled_ ? this->filteredValue_
									  : this->value_;
	}

protected:

	MovingAverageValue<Type> filteredValue_;
	bool maintainEnabled_{false};


};

#endif //UAVAP_OVERRIDABLEVALUE_H
