//
// Created by Mirco Theile on 1/9/24.
//

#ifndef FORWARDINGLOCALPLANNERPARAMS_H
#define FORWARDINGLOCALPLANNERPARAMS_H

#include <string>
#include "cpsCore/Configuration/Parameter.hpp"


struct ForwardingLocalPlannerParams
{
    Parameter<std::string> ipcTarget = {"trajectory", "ipc_target", false};

    template <class Configurator>
    inline void
    configure(Configurator& c)
    {
        c & ipcTarget;
    }
};

#endif //FORWARDINGLOCALPLANNERPARAMS_H
