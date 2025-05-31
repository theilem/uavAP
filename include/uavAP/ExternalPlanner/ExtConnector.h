//
// Created by Mirco Theile on 29/5/25.
//

#ifndef EXTCONNECTOR_H
#define EXTCONNECTOR_H

#include <boost/signals2/connection.hpp>
#include <cpsCore/cps_object>

#include "cpsCore/Utilities/IDC/IDCSender.h"
#include "cpsCore/Utilities/IPC/Publisher.h"

class ISensingIO;
class DataPresentation;
class IPC;
class IDC;

struct ExtConnectorParams
{
    template <typename Configurator>
    void
    configure(Configurator& c)
    {
    }
};

class ExtConnector: public ConfigurableObject<ExtConnectorParams>,
                      public AggregatableObject<IPC, IDC, DataPresentation, ISensingIO>,
                      public IRunnableObject
{
public:
    static constexpr auto typeId = "ext_connector";
    ExtConnector() = default;
    ~ExtConnector() override = default;
    bool
    run(RunStage stage) override;
private:
    void
    onExtPacket(const Packet& packet);
    IDCSender extSender_;
    Publisher<Packet> missionControlPublisher_;
    Publisher<Packet> commsPublisher_;
    boost::signals2::connection sensorSubscription_;
};

#endif //EXTCONNECTOR_H
