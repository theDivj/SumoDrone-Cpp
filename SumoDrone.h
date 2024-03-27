#pragma once
#include <iostream>
#include <cstdlib>
#include <string>
#include <vector>
#include <limits>
#include <fstream>

#include <libsumo/libsumo.h>

#include "GlobalFlags.h"
#include "ChargeHubs.h"
#include "ControlCentre.h"
#include "dSimulation.h"

class SumoDrone {
public:
    bool briefStatistics;   // = false
    int maxEVs;             // = numeric_limits<int>::max();
    std::string sumoCmd;
    std::string runstring;
    
    GlobalFlags* gg;

    SumoDrone();

    ~SumoDrone();

    void loop() const;

    GlobalFlags* parseRunstring(int argc, char* argv[]);
};
