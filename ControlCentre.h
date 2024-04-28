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
#include "urgency.h"

class Drone;

class ControlCentre
{
    friend class urgency;

protected:
    static inline double wEnergy;
    static inline double wUrgency;
    static inline double proximityRadius;

private:
    static inline int maxDrones;
    static inline bool zeroDrone;
    static inline std::unordered_map< EV*, double> requests;
    static inline std::unordered_map< EV*, Drone*> allocatedEV;
    static inline std::unordered_map< EV*, double> startChargeEV;
    static inline std::unordered_map<Drone*,  EV*> allocatedDrone;
    static inline std::unordered_set<Drone*> needChargeDrones;
    static inline int spawnedDrones;
    static inline int fullChargeTolerance;

public:
    int insertedDummies;

    static inline std::unordered_set<Drone*> freeDrones;

    ControlCentre(double wEnergy, double wUrgency, double proximityRadius, int maxDrones, int fullChargeTolerance);

    ControlCentre() = default;

    void allocate(Drone* drone,  EV* ev);
    
    void allocateDrones();

    bool chargeCanComplete(EV* ev);

    //std::set < urgency, decltype(urgencyCmp)* > calcUrgency();

    std::tuple<std::string, double, bool> findEdgePos(std::string evID, double deltaPos);

    libsumo::TraCIPosition findRendezvousXY( EV* ev, Drone* drone);

    //std::pair<std::unordered_set< EV*>, double> getNeighboursNeedingCharge( EV* ev, bool firstCall);

    void notifyDroneState(Drone* drone);

    void notifyEVState( EV* ev, EVState evState, Drone* drone, double capacity);

    void printDroneStatistics(bool brief, std::string version, std::string runstring);

    void requestCharge( EV* ev, double capacity, double requestedWh);

    void setMaxDrones(int pmaxDrones) {
        maxDrones = pmaxDrones;
        syncSpawnedDrones();
    }

    void syncSpawnedDrones() { // if we'd generated drones from POI we need to update our spawnedDrone count
        spawnedDrones = Drone::getIDCount();
    }

    void tidyDrones();

    void update();
};
//std::pair<std::unordered_set< EV*>, double> getNeighboursNeedingCharge( EV* ev, bool firstCall);

