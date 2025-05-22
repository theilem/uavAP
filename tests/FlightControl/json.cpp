//
// Created by Mirco Theile on 20/5/25.
//

#include <cpsCore/Utilities/Test/TestInfo.h>
#include <cpsCore/Configuration/JsonPopulator.h>

#include "uavAP/FlightControl/FlightControlHelper.h"

TEST_CASE("Json Populator Test -- Flight Control Helper")
{
    auto config = JsonPopulator::populateContainer<FlightControlHelper>();
    // std::cout << config.getString() << std::endl;
}