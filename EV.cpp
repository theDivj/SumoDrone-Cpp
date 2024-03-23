#include <cstdlib>
#include <iostream>
#include <string>
#include <iomanip>

#include "EV.h"
#include "Drone.h"
#include "GlobalFlags.h"


using namespace libsumo;
using namespace std;

// initialize class variables
double EV::chargeNeededThreshold = 30000.0;
double EV::chargeDoneThreshold = 32000.0;
double EV::evChargeRequestWh = 2000.0;
double EV::pRandomVariation = 0.30;
double EV::kmPerWh = 6.5 / 1000.0;

int EV::evCount = 0;
int EV::evChargeSteps = 0;
double EV::evChargeGap = 0.0;
int EV::evChargeCount = 0;

EV::EV(const std::string& evID, double kmPerWh) {

    if (kmPerWh <= 0.0) {
        this->myKmPerWh = EV::kmPerWh;
    }
    else {
        this->myKmPerWh = kmPerWh;
    }
    myID = evID;
    myState = EVState::DRIVING;
    myPosition.x = 0.0; myPosition.y = 0.0;
    myRendezvous.x = 0.0; myRendezvous.y = 0.0;
    myDrone = nullptr;
    myColour = Vehicle::getColor(myID);
    myChargeCount = 0;
    myChargeSteps = 0;
    myChaseSteps = 0;
    myCapacity = EV::chargeDoneThreshold;
    
    myChargeNeededThreshold = EV::chargeNeededThreshold;
    myevChargeRequestWh = EV::evChargeRequestWh;
 
    //setEVOverrides(myID);

    if (GlobalFlags::usingRandom()) {
        double variation = pRandomVariation * myevChargeRequestWh;
        variation *= ((2. * (rand()/double(RAND_MAX))) - 1.);
        myChargeNeededThreshold = myChargeNeededThreshold + variation + 1000. * rand()/double(RAND_MAX) - 500;
        myevChargeRequestWh += variation;
    }
    myChargeDone = myChargeNeededThreshold + myevChargeRequestWh;
    myLastChargeRequest = myevChargeRequestWh;
    evCount++;
}


void EV::setEVOverrides(string vehID) {
    // check to see if we have an override defined for charge request - could be in type or vehicle definition - vehicle takes precedence
    string vType = Vehicle::getTypeID(vehID);
    string overrideChargeWh = VehicleType::getParameter(vType, "chargeRequestWh");
    string vehicleOverrideChargeWh = Vehicle::getParameter(vehID, "chargeRequestWh");
    double oWh = 0.;
    if (overrideChargeWh.size() > 1)
        oWh = stod(overrideChargeWh);
    if (vehicleOverrideChargeWh.size() > 1)
        oWh = stod(vehicleOverrideChargeWh);
    if (oWh > 1)
        myevChargeRequestWh = oWh;

    string overrideChargeRequestThresholdWh = VehicleType::getParameter(vType, "chargeRequestThresholdWh");
    string vehicleChargeRequestThresholdWh = Vehicle::getParameter(vehID, "chargeRequestThresholdWh");
    oWh = 0.;
    if (overrideChargeRequestThresholdWh.size() > 1)
        oWh = stod(overrideChargeRequestThresholdWh);
    if (vehicleChargeRequestThresholdWh.size() > 1)
        oWh = stod(vehicleChargeRequestThresholdWh);
    if (oWh > 5000)
        myChargeNeededThreshold = oWh;
}

void EV::stopCharging(double remainingCharge) {      // state change triggered by drone or ev leaving

    if (myState ==EVState::CHARGINGFROMDRONE) {
        if (remainingCharge > 0) {
            myState =EVState::CHARGEREQUESTED;
            GlobalFlags::cc->notifyEVState(this,EVState::CHARGEBROKENOFF, myDrone, myCapacity);
            GlobalFlags::cc->requestCharge(this, myCapacity, remainingCharge);
            myDrone = nullptr;
        }
        else {
            myCapacity = std::stod(Vehicle::getParameter(myID, "device.battery.actualBatteryCapacity"));
            myState =EVState::DRIVING;
            GlobalFlags::cc->notifyEVState(this, myState, myDrone, myCapacity);
            myDrone = nullptr;
        }
    }
    else {
        myState =EVState::DRIVING;
        myDrone = nullptr;
    }

}

void EV::update() {

    switch (myState) {
    case EVState::DRIVING:
        if ((myChargeCount < 1) || (!GlobalFlags::myOnlyChargeOnce)) {
            myCapacity = std::stod(Vehicle::getParameter(myID, "device.battery.actualBatteryCapacity"));
            if (myCapacity < myChargeNeededThreshold) {
                setMyPosition();
                Vehicle::setColor(myID, TraCIColor(255, 0, 0, 255));
                myState = EVState::CHARGEREQUESTED;
                setLastChargeRequest();
                GlobalFlags::cc->requestCharge(this, myCapacity, myLastChargeRequest);
            }
        }
        break;

    case EVState::CHARGEREQUESTED:
        if (myDrone != nullptr) {
            if (GlobalFlags::myModelRendezvous) {
                myDrone->update(myRendezvous);
                myChaseSteps = 0;
                myState = EVState::WAITINGFORRENDEZVOUS;
            }
            else {
                setMyPosition();
                myDrone->update(myPosition);
                myState = EVState::WAITINGFORDRONE;
            }
        }
        break;

    case EVState::WAITINGFORRENDEZVOUS:
        if (myDrone != nullptr) {
            if (myDrone->update(myRendezvous).first) {
                myState =EVState::WAITINGFORDRONE;
            }
        }
        break;

    case EVState::WAITINGFORDRONE:
        setMyPosition();
        myChaseSteps++;
        if (myDrone != nullptr) {
            if (myDrone->update(myPosition).first) {
                if (myState ==EVState::WAITINGFORDRONE) {   // myDrone.update could have called EV.stopCharging
                    myDrone->notifyChase(true, myChaseSteps);
                    Vehicle::setColor(myID, TraCIColor(0, 255, 0, 255));
                    myState =EVState::CHARGINGFROMDRONE;
                    myCapacity = std::stod(Vehicle::getParameter(myID, "device.battery.actualBatteryCapacity"));
                    GlobalFlags::cc->notifyEVState(this, myState, myDrone, myCapacity);
                    myChargeDone = myCapacity + myLastChargeRequest;
                }
                else {
                    myDrone->notifyChase(false, myChaseSteps);
                    Vehicle::setColor(myID, myColour);
                    myDrone = nullptr;
                }
            }
        }
        break;

    case EVState::CHARGINGFROMDRONE:
        setMyPosition();
        myCapacity = std::stod(Vehicle::getParameter(myID, "device.battery.actualBatteryCapacity"));
        {
            pair<bool, double> upd = myDrone->update(myPosition);
            if (!upd.first) {   // either charge is finished or drone has broken off
                if (myState ==EVState::CHARGEREQUESTED) { //  Drone broke off before charge completed
                    myDrone = nullptr;
                    break;
                }
                if (myState ==EVState::DRIVING) {
                    Vehicle::setColor(myID, myColour);
                    myDrone = nullptr;
                    myChargeCount++;
                    break;
                }
            }
            else {
                myCapacity += upd.second;
                Vehicle::setParameter(myID, "device.battery.actualBatteryCapacity", to_string(myCapacity));
                myChargeSteps++;
            }
        }
        break;

    case EVState::LEFTSIMULATION:
        captureStats();
        if (myDrone != nullptr) {
            GlobalFlags::cc->notifyEVState(this, myState, myDrone, myCapacity);
            myDrone->notifyEVFinished(myState);
            myDrone = nullptr;
        }
        else {
            GlobalFlags::cc->notifyEVState(this, myState, myDrone, myCapacity);
        }
        myState =EVState::NULLSTATE;
        break;

    case EVState::NULLSTATE:
        break;
    }
}