#pragma once
#include <unordered_map>
#include <vector>
#include <string>
#include <libsumo/libsumo.h>

class EV;

class dSimulation {
public:
    static inline int maxEVs;
    static inline double stepSecs;
    static inline bool useChargeHubs;
    static inline int timeStep;

    static inline std::unordered_map<std::string,EV*> EVs;

    dSimulation(const std::vector<std::string> sumoCmd, int maxEVs);

    dSimulation() = default;

    ~dSimulation() {
        libsumo::Simulation::close();
        dSimulation::EVs.clear();
    }

    static bool dStep();
};
