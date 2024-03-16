#pragma once
#include <algorithm>
#include <cmath>
#include <iostream>
#include <libsumo/libsumo.h>
#include <limits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <vector>

#include "GlobalFlags.h"
#include "EVState.h"

class urgency;
class Drone;

#include "urgency.h"

class ControlCentre
{
public:
    static inline double wEnergy;
    static inline double wUrgency;
    static inline double proximityRadius;
    static inline int maxDrones;
    static inline std::string droneType;

    static inline std::unordered_map< EV*, double> requests;
    static inline std::unordered_map< EV*, Drone*> allocatedEV;
    static inline std::unordered_map< EV*, double> startChargeEV;
    static inline std::unordered_map<Drone*,  EV*> allocatedDrone;
    static inline std::unordered_set<Drone*> freeDrones;
    static inline std::unordered_set<Drone*> needChargeDrones;
    static inline int spawnedDrones;


    ControlCentre(double wEnergy, double wUrgency, double proximityRadius, int maxDrones, std::string droneType = "ehang184");

    ControlCentre() = default;

    void allocate(Drone* drone,  EV* ev);
    
    void allocateDrones();

    //std::set < urgency, decltype(urgencyCmp)* > calcUrgency();

    std::tuple<std::string, double, bool> findEdgePos(std::string evID, double deltaPos);

    libsumo::TraCIPosition findRendezvousXY( EV* ev, Drone* drone);

    //std::pair<std::unordered_set< EV*>, double> getNeighboursNeedingCharge( EV* ev, bool firstCall);

    void notifyDroneState(Drone* drone);

    void notifyEVState( EV* ev, EVState evState, std::string droneID, double capacity);

    void printDroneStatistics(bool brief, std::string version);

    void requestCharge( EV* ev, double capacity, double requestedWh);

    void update();
};
//std::pair<std::unordered_set< EV*>, double> getNeighboursNeedingCharge( EV* ev, bool firstCall);

