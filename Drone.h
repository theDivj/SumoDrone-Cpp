#pragma once
#include <libsumo/libsumo.h>
#include <string>

#include "ChargeHubs.h"
#include "DroneState.h"
#include "EVState.h"
#include "DroneType.h"

class EV;

class Drone
{

public:
    static int droneIDCount;
    static DroneType* d0Type;

private:
    static bool parkAtHome;                          

    static bool dummyEVCreated;       // whether we have created the dummy ev vehicle type, used to get charging station output
 
    static bool printedType;

    std::string myID;
    libsumo::TraCIPosition myPosition;
    libsumo::TraCIPosition myParkPosition;
    hubLocation myParkEP;
    bool myViableCharge;
    DroneState myState;
    EV* myEV;

public:
    DroneType* myDt;
    double myCharge;
    double myFlyingCharge;

    // logging
    int myFlyingCount;              // used to compute distance travelled
    int myFullCharges;              // count of complete charges
    int myBrokenCharges;             // count of charges broken off - by me out of charge
    int myBrokenEVCharges;          // count of charges broken off - by EV(leaving)
    int myEVChargingCount;            // Count of steps I've been charging EVs
    int myChargeMeFlyingCount;      // wh i've charged my flying battery
    int myChargeMeCount;            // wh i've used charging my EV charging battery
    int myChaseCount;               // count of complete chases - ie got from rendezvous to ev
    int myBrokenChaseCount;         // count of broken chases where we didn't get to ev before it left sim
    int myChaseSteps;               // count of steps in all complete chases - used with myChaseCount to compute average
    double myRequestedCharge;       // the amount of charge requested by the EV
    bool myDummyEVInserted;         // whether the dummy EVs have been inserted
 
    Drone(libsumo::TraCIPosition pos, std::string ID = "", DroneType* DT = nullptr);

    Drone() = default;

    static int getIDCount() {
        return droneIDCount;
    }

    static void setDroneType(bool useOneBattery, std::string droneType);

    static int setDroneTypeFromPOI(bool useOneBattery, bool zeroDrone);   // returns how many drones were created                                       

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

    void logLine(std::string activity) const;
                                                   
    void notifyChase(bool chaseOK, int chaseSteps);

    void notifyEVFinished(EVState evState);

    void park();

    void parkingUpdate();

    void setMyParkPosition();

    void setViableCharge();

    std::pair<bool,double> update(libsumo::TraCIPosition pos);

    bool usePower(std::string mode);

    bool viable() const;
};