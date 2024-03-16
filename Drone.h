#pragma once
#include <libsumo/libsumo.h>
#include <string>
#include "ChargeHubs.h"
#include "DroneState.h"
#include "EVState.h"

class EV;

class Drone
{
public:

    static int droneIDCount;
    static bool parkAtHome;                          

    //  Drone specific class variables  - overridden by setDroneType                                                               
    static double droneKMperh;      // drone cruising speed - will be overridden by global / runstring value
    static double droneChargeWh;           // capacity of battery used to charge ev's  - based on Ehang 184 load capacity
    static double droneFlyingWh;           // capacity of battery used to power drone
    static double droneFlyingWhperTimeStep;  // power usage based on Ehang 184 which has battery capacity of 14.4 KW giving 23 mins flight time
    static double droneChargeContingencyp;   // minimum contingency level %
    static double droneChargeViablep;        // minimum viable level %
    static double WhEVChargeRatePerTimeStep;      // 25KW   rate of vehicle charge from drone(adjust for timeStep when simulation starts)
    static double WhDroneRechargePerTimeStep;     // 75KW   rate of drone charge when parked(adjust for timeStep when simulation starts)
    // Derived class variables
    static double droneMperSec;
    static double droneStepMperTimeStep;                           // How far(metres) the drone will travel in one time step(adjust for timeStep when simulation starts)
    static double droneStepM2;    // precompute - used in distance calculations(adjust for timeStep when simulation starts)
    static double minDroneCharge;  // Thresholds to break off charging / flying
    static double minDroneFlyingWh;
    static double viableDroneCharge;    // thresholds to allow allocation - ie enough charge to be useful
    static double viableDroneFlyingWh;
    
    static bool dummyEVCreated;       // whether we have created the dummy ev vehicle type, used to get charging station output

    std::string myID;
    libsumo::TraCIPosition myPosition;
    libsumo::TraCIPosition myParkPosition;
    hubLocation myParkEP;
    double myCharge;
    double myFlyingCharge;
    bool myViableCharge;
    DroneState myState;
    EV* myEV;

    // logging
    int myFlyingCount;            // used to compute distance travelled
    int myFullCharges;             // count of complete charges
    int myBrokenCharges;           // count of charges broken off - by me out of charge
    int myBrokenEVCharges;         // count of charges broken off - by EV(leaving)
    double myFlyingWh;              // wH i've used flying
    double myChargingWh;            // wH i've used charging EVs
    int myChargeMeFlyingCount;   // wh i've charged my flying battery
    int myChargeMeCount;         // wh i've used charging my EV charging battery
    int myChaseCount;              // count of complete chases - ie got from rendezvous to ev
    int myBrokenChaseCount;        // count of broken chases where we didn't get to ev before it left sim
    int myChaseSteps;              // count of steps in all complete chases - used with myChaseCount to compute average
    double myRequestedCharge;         // the amount of charge requested by the EV
    bool myDummyEVInserted;     // whether the dummy EVs have been inserted

    void setDroneType(std::string droneType);
                                        
    Drone(libsumo::TraCIPosition xypos, std::string droneType);

    Drone() = default;
    //Drone(double myc = 0, double myf = 0, double myv, DroneState mys = DroneState::NULLSTATE, EV* mye = nullptr) :
      //          myCharge{myc}, myFlyingCharge{myf},  myViableCharge(myv), myState(mys), myEV(mye) {}

   bool operator<(const Drone& other) const {
        return myID < other.myID; // need to substring for number comparison
    }

    std::string toString() const {
        return myID;
    }

    bool allocate(EV* ev, double requestedCharge);

    void chargeMe();

    void createDummyEV();

    void dummyEVHide();

    void dummyEVInsert();

    bool fly(libsumo::TraCIPosition pos);

    std::string getID() const { return myID; }

    libsumo::TraCIPosition getMyPosition() const { return myPosition; }

    void logLine(std::string activity);
                                                   
    void notifyChase(bool chaseOK, int chaseSteps);

    void notifyEVFinished(EVState evState);

    void park();

    void parkingUpdate();

    void setMyParkPosition();

    void setViableCharge();

    static void stepSecsAdjust(double stepSecs);

    std::pair<bool,double> update(libsumo::TraCIPosition pos);

    bool usePower(std::string mode);

    bool viable();
};