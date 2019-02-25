/*
 * ManualWindEstimation.h
 *
 *  Created on: Feb 8, 2019
 *      Author: mirco
 */

#ifndef UAVAP_FLIGHTANALYSIS_WINDESTIMATION_MANUALWINDESTIMATION_H_
#define UAVAP_FLIGHTANALYSIS_WINDESTIMATION_MANUALWINDESTIMATION_H_
#include <uavAP/Core/IPC/Publisher.h>
#include <uavAP/Core/Object/IAggregatableObject.h>
#include <uavAP/Core/Object/ObjectHandle.h>
#include <uavAP/Core/Runner/IRunnableObject.h>
#include <uavAP/FlightAnalysis/WindEstimation/IWindEstimation.h>
#include <uavAP/FlightAnalysis/WindEstimation/WindInfo.h>

class IPC;
class Packet;

class ManualWindEstimation : public IAggregatableObject, public IRunnableObject, public IWindEstimation
{
public:

	bool
	configure(const Configuration& config);

	static std::shared_ptr<ManualWindEstimation>
	create(const Configuration& config);

	void
	notifyAggregationOnUpdate(const Aggregator& agg) override;

	bool
	run(RunStage stage) override;

	WindInfo
	getWindInfo() const override;

private:

	void
	onPacket(const Packet& packet);

	WindInfo currentWind_;

	ObjectHandle<IPC> ipc_;
	Publisher publisher_;

};



#endif /* UAVAP_FLIGHTANALYSIS_WINDESTIMATION_MANUALWINDESTIMATION_H_ */
