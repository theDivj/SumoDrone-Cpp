#pragma once
#include <unordered_map>
#include <vector>
#include <string>
#include <libsumo/libsumo.h>

class EV;

class dSimulation {

    int maxEVs;
    std::unordered_map<std::string,EV*> EVs;
    bool usingSumogui;
    bool useChargeHubs;
    double stepSecs;
    int timeStep;

public:
    dSimulation(const std::vector<std::string> sumoCmd, int maxEVs);

    dSimulation() = default;

    ~dSimulation() {
        libsumo::Simulation::close();
        EVs.clear();
    }

    bool dStep();

    bool getUseChargeHubs() {
        return useChargeHubs;
    }

    double getStepSecs() {
        return stepSecs;
    }

    double getTimeStep() {
        return timeStep;
    }

    void setMaxEvs(int pmaxEvs) {
        maxEVs = pmaxEvs;
    }

};
