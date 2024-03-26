#pragma once
#include <iostream>
#include <libsumo/libsumo.h>

#include "EVState.h"
#include "Drone.h"

class EV
{
    friend class ControlCentre;

protected:
    static double chargeNeededThreshold;
    static double chargeDoneThreshold;
    static double evChargeRequestWh;
    static double pRandomVariation;
    static double kmPerWh;
    static int evCount;
    static int evChargeSteps;
    static double evChargeGap;
    static int evChargeCount;

private:
    double myKmPerWh;
    std::string myID;
    EVState myState;
    libsumo::TraCIPosition myPosition;
    libsumo::TraCIPosition myRendezvous;
    Drone* myDrone = nullptr;
    libsumo::TraCIColor myColour;
    int myChargeSteps;
    int myChaseSteps;
    double myCapacity;
    double myChargeNeededThreshold;
    double myevChargeRequestWh;
    int myChargeCount;
    double myChargeDone;
    double myLastChargeRequest;

public:
    EV(const std::string& evID, double kmPerWh = EV::kmPerWh);

    EV() = default;

    bool operator<(const  EV& other) const {
        return myID < other.myID;
    }

    std::string toString() const {
        return myID;
    }

    void allocate(Drone* drone, libsumo::TraCIPosition rvPos) { // save the drone/ev relationship and set rendezvous position - may be N
        myRendezvous = rvPos;
        myDrone = drone;
    }

    void captureStats() const {    // Add statistics for this vehicle into class variables - called when EV leaves
        EV::evChargeGap += (myChargeDone - myCapacity);
        EV::evChargeCount += myChargeCount;
        EV::evChargeSteps += myChargeSteps;
   }

    std::string getID() const {    // getter function for EV identity
        return myID;
    }

    static double getEVKmPerWh() {  // getter for EV default
        return EV::kmPerWh;
    }

    double getMyKmPerWh() const {  // getter for my average usage
        return myKmPerWh;
    }

    void setEVOverrides(std::string vehID);   // pick up any overrides from add.xml file(s)

    libsumo::TraCIPosition getMyPosition() const {   // getter function for x, y position
        return myPosition;
    }

    void leftSimulation() {    // State change
        myState = EVState::LEFTSIMULATION;
    }

    void setLastChargeRequest() {   //  Work out how much charge (in Wh) is needed"""
        // currently varies if randomseed is passed in runString - otherwise just the EV value
        myLastChargeRequest = myevChargeRequestWh;
        myChargeDone = myCapacity + myevChargeRequestWh;  // not actually used at the moment - drone cuts off after delivering requested charge
    }

    void setMyPosition() { // get real EV position from simulation and set my variable
        if (myState == EVState::WAITINGFORRENDEZVOUS)       // never called from this state so
            myDrone->notifyChase(false, myChaseSteps);          // must be failed chase
        libsumo::TraCIPosition pos = libsumo::Vehicle::getPosition(myID);
        myPosition.x = pos.x;
        myPosition.y = pos.y;
    }

    void stopCharging(double remainingCharge);      // state change triggered by drone or ev leaving

    void update();

    bool operator==(const  EV& otherEV) const
    {
        if (this->myID == otherEV.myID) return true;
        else return false;
    }

    struct evIDHash
    {
        size_t operator()(const  EV& ev) const
        {
            size_t IDHash = std::hash<std::string>()(ev.myID);
            return IDHash;
        }
    };

};
