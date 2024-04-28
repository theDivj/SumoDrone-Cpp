#include "DroneType.h"
#include <iostream>
#include <iomanip>

DroneType::DroneType() {
    droneKMperh = 60.0;                          // drone cruising speed - will be overridden by global / runstring value
    droneChargeWh = 30000.0;                       // capacity of battery used to charge ev's  - based on Ehang 184 load capacity
    droneFlyingWh = 14400.0;                       // capacity of battery used to power drone
    droneFlyingWhperTimeStep = droneFlyingWh / (23 * 60.); ;  // power usage based on Ehang 184 which has battery capacity of 14.4 KW giving 23 mins flight time
    droneChargeContingencyp = .05;           // minimum contingency level %
    droneChargeViablep = .3;                      // minimum viable level %
    WhEVChargeRatePerTimeStep = 25000 / 3600.;      // 25KW   rate of vehicle charge from drone(adjust for timeStep when simulation starts)
    WhDroneRechargePerTimeStep = 75000 / 3600.;     // 75KW   rate of drone charge when parked(adjust for timeStep when simulation starts)

    droneImageFile = "drone.png";
    droneColour = { 0, 0, 255, 255 };
    droneWidth = 10.0;
    droneHeight = 10.0;
    useOneBattery = false;

    // derived variables
    droneMperSec = droneKMperh / 3.6;
    droneStepMperTimeStep = droneMperSec;                        // How far(metres) the drone will travel in one time step(adjust for timeStep when simulation starts)
    droneStepM2 = pow(droneStepMperTimeStep, 2);                 // precompute - used in distance calculations(adjust for timeStep when simulation starts)
    minDroneCharge = droneChargeContingencyp * droneChargeWh;    // Thresholds to break off charging / flying
    minDroneFlyingWh = droneChargeContingencyp * droneFlyingWh;
    viableDroneCharge = droneChargeViablep * droneChargeWh;      // thresholds to allow allocation - ie enough charge to be useful
    viableDroneFlyingWh = droneChargeViablep * droneFlyingWh;
}

void DroneType::setDerived(double stepSecs = 1.0) {
    //droneMperSec = (droneKMperh / 3.6);
    droneStepMperTimeStep = droneMperSec * stepSecs;                        // How far(metres) the drone will travel in one time step(adjust for timeStep when simulation starts)
    droneStepM2 = pow(droneStepMperTimeStep, 2);                 // precompute - used in distance calculations(adjust for timeStep when simulation starts)

    droneFlyingWhperTimeStep *= stepSecs;
    WhEVChargeRatePerTimeStep *= stepSecs;
    WhDroneRechargePerTimeStep *= stepSecs;

    minDroneCharge = droneChargeContingencyp * droneChargeWh;    // Thresholds to break off charging / flying
    minDroneFlyingWh = droneChargeContingencyp * droneFlyingWh;
    viableDroneCharge = droneChargeViablep * droneChargeWh;      // thresholds to allow allocation - ie enough charge to be useful
    viableDroneFlyingWh = droneChargeViablep * droneFlyingWh;
}

std::ostream& operator<<(std::ostream& os, DroneType const& dt) {
    os << std::setprecision(6);
    os << "droneKMperh\t" << dt.droneKMperh << "\n";                 // drone cruising speed - will be overridden by global / runstring value
    os << "droneChargeWh\t" << dt.droneChargeWh << "\n";               // capacity of battery used to charge ev's  - based on Ehang 184 load capacity
    os << "droneFlyingWh\t" << dt.droneFlyingWh << "\n";               // capacity of battery used to power drone
    os << "droneFlyingWhperTimeStep\t" << dt.droneFlyingWhperTimeStep << "\n";    // power usage based on Ehang 184 which has battery capacity of 14.4 KW giving 23 mins flight time
    os << "droneChargeContingencyp\t" << dt.droneChargeContingencyp << "\n";     // minimum contingency level %
    os << "droneChargeViablep\t" << dt.droneChargeViablep << "\n";          // minimum viable level %
    os << "WhEVChargeRatePerTimeStep\t" << dt.WhEVChargeRatePerTimeStep << "\n";   // 25KW   rate of vehicle charge from drone(adjust for timeStep when simulation starts)
    os << "WhDroneRechargePerTimeStep\t" << dt.WhDroneRechargePerTimeStep << "\n";  // 75KW   rate of drone charge when parked(adjust for timeStep when simulation starts)

    os << "droneImageFile\t" << dt.droneImageFile << "\n";
    os << "droneColour\t" << dt.droneColour.getString() << "\n";
    os << "droneWidth\t" << dt.droneWidth << "\n";
    os << "droneHeight\t" << dt.droneHeight << "\n";

    // derived variables
    os << "droneMperSec\t" << dt.droneMperSec << "\n";
    os << "droneStepMperTimeStep\t" << dt.droneStepMperTimeStep << "\n";   // How far(metres) the drone will travel in one time step(adjust for timeStep when simulation starts)
    os << "droneStepM2\t" << dt.droneStepM2 << "\n";             // precompute - used in distance calculations(adjust for timeStep when simulation starts)
    os << "minDroneCharge\t" << dt.minDroneCharge << "\n";          // Thresholds to break off charging / flying
    os << "minDroneFlyingWh\t" << dt.minDroneFlyingWh << "\n";
    os << "viableDroneCharge\t" << dt.viableDroneCharge << "\n";       // thresholds to allow allocation - ie enough charge to be useful
    os << "viableDroneFlyingWh\t" << dt.viableDroneFlyingWh << "\n";
    os << "useOneBattery\t" << dt.useOneBattery << "\n";
    return os;
}
