//
// Created by mirco on 09.03.21.
//

#ifndef UAVAP_ISIGNALGENERATOR_H
#define UAVAP_ISIGNALGENERATOR_H

class ISignalGenerator
{
public:

	static constexpr auto typeId = "signal_generator";

	virtual ~ISignalGenerator() = default;

	virtual void
	initialize() = 0;

	virtual FloatingType
	getValue() = 0;

};

#endif //UAVAP_ISIGNALGENERATOR_H
