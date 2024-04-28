#pragma once
#include <string>
#include "libsumo/libsumo.h"

class DroneType {
public:
    double droneKMperh;                 // drone cruising speed - will be overridden by global / runstring value
    double droneChargeWh;               // capacity of battery used to charge ev's  - based on Ehang 184 load capacity
    double droneFlyingWh;               // capacity of battery used to power drone
    double droneFlyingWhperTimeStep;    // power usage based on Ehang 184 which has battery capacity of 14.4 KW giving 23 mins flight time
    double droneChargeContingencyp;     // minimum contingency level %
    double droneChargeViablep;          // minimum viable level %
    double WhEVChargeRatePerTimeStep;   // 25KW   rate of vehicle charge from drone(adjust for timeStep when simulation starts)
    double WhDroneRechargePerTimeStep;  // 75KW   rate of drone charge when parked(adjust for timeStep when simulation starts)

    std::string droneImageFile;
    libsumo::TraCIColor droneColour;
    double droneWidth;
    double droneHeight;
    bool useOneBattery;             // whether to use charge battery for flying and charging

    // derived variables
    double droneMperSec;
    double droneStepMperTimeStep;   // How far(metres) the drone will travel in one time step(adjust for timeStep when simulation starts)
    double droneStepM2;             // precompute - used in distance calculations(adjust for timeStep when simulation starts)
    double minDroneCharge;          // Thresholds to break off charging / flying
    double minDroneFlyingWh;
    double viableDroneCharge;       // thresholds to allow allocation - ie enough charge to be useful
    double viableDroneFlyingWh;

    DroneType();

    void setDerived(double stepSecs);
};
std::ostream& operator<<(std::ostream& os, DroneType const& dt);