#include "Drone.h"
#include "EV.h"
#include <string>
#include "GlobalFlags.h"
#include <cmath>
#include <exception>
#include <iomanip>

using namespace libsumo;
using namespace std;


Drone::Drone(TraCIPosition pos, string droneType = "ehang184") {
    Drone::droneIDCount += 1;
    Drone::setDroneType(droneType);   // should we be setting this on each drone call - it sets the class variables
    myID = "d" + to_string(Drone::droneIDCount);
    myPosition = pos;
    myParkPosition = myPosition;
    //myParkEP;
    myCharge = Drone::droneChargeWh;
    myFlyingCharge = Drone::droneFlyingWh;
    myViableCharge = true;
    myState = DroneState::NULLSTATE;
    myEV = nullptr;

    // logging variables
    myFlyingCount = 0;            // used to compute distance travelled
    myFullCharges = 0;             // count of complete charges
    myBrokenCharges = 0;           // count of charges broken off - by me out of charge
    myBrokenEVCharges = 0;         // count of charges broken off - by EV(leaving)
    myFlyingWh = 0.0;              // wH i've used flying
    myChargingWh = 0.0;            // wH i've used charging EVs
    myChargeMeFlyingCount = 0;   // wh i've charged my flying battery
    myChargeMeCount = 0;         // wh i've used charging my EV charging battery
    myChaseCount = 0;              // count of complete chases - ie got from rendezvous to ev
    myBrokenChaseCount = 0;        // count of broken chases where we didn't get to ev before it left sim
    myChaseSteps = 0;              // count of steps in all complete chases - used with myChaseCount to compute average
    myRequestedCharge = 0;         // the amount of charge requested by the EV
    myDummyEVInserted = false;     // whether the dummy EVs have been inserted
        // finally create the POI representing our drone
    POI::add(myID, pos.x, pos.y, { 0, 0, 255, 255 }, "", 250, "drone.png", 10, 10);

    if (GlobalFlags::ss->useChargeHubs and not dummyEVCreated)  
        createDummyEV();
}

// initialise class variables
int Drone::droneIDCount = 0;
bool Drone::parkAtHome = false;

double Drone::droneKMperh = 60.0;      // drone cruising speed - will be overridden by global / runstring value
double Drone::droneChargeWh = 30000.;           // capacity of battery used to charge ev's  - based on Ehang 184 load capacity
double Drone::droneFlyingWh = 14400.;           // capacity of battery used to power drone
double Drone::droneFlyingWhperTimeStep = droneFlyingWh / (23 * 60.);  // power usage based on Ehang 184 which has battery capacity of 14.4 KW giving 23 mins flight time
double Drone::droneChargeContingencyp = 0.05;   // minimum contingency level %
double Drone::droneChargeViablep = 0.3;        // minimum viable level %
double Drone::WhEVChargeRatePerTimeStep = 25000. / 3600;      // 25KW   rate of vehicle charge from drone(adjust for timeStep when simulation starts)
double Drone::WhDroneRechargePerTimeStep = 75000. / 3600;     // 75KW   rate of drone charge when parked(adjust for timeStep when simulation starts)
// Derived class variables
double Drone::droneMperSec = Drone::droneKMperh / 3.6;
double Drone::droneStepMperTimeStep = Drone::droneMperSec;                           // How far(metres) the drone will travel in one time step(adjust for timeStep when simulation starts)
double Drone::droneStepM2 = pow(Drone::droneStepMperTimeStep,2);    // precompute - used in distance calculations(adjust for timeStep when simulation starts)
double Drone::minDroneCharge = Drone::droneChargeContingencyp * Drone::droneChargeWh;  // Thresholds to break off charging / flying
double Drone::minDroneFlyingWh = Drone::droneChargeContingencyp * Drone::droneFlyingWh;
double Drone::viableDroneCharge = Drone::droneChargeViablep * Drone::droneChargeWh;    // thresholds to allow allocation - ie enough charge to be useful
double Drone::viableDroneFlyingWh = Drone::droneChargeViablep * Drone::droneFlyingWh;
bool Drone::dummyEVCreated = false;


void Drone::setDroneType(std::string droneType="ehang184") {  // Support different drone definitions - initially to give us a drone that doesn't need charging"""

    Drone::droneKMperh = GlobalFlags::getDroneSpeed(); // get speed override if any

    // EV charging battery size is constrained by drone carrying capacity * average battery energy density(currently ~150Wh / Kg)
    if (droneType == "ehang184x") {  // ehang 184 with artificially increased battery sizes so they don't need recharging
        Drone::droneChargeWh = 3000000.0;    // 100 * actual
        Drone::droneFlyingWh = 14400000.0;
        Drone::droneFlyingWhperTimeStep = 14400 / (23 * 60.);
        Drone::droneChargeContingencyp = 0.05;              // minimum contingency level %
        Drone::droneChargeViablep = 0.3;                    // minimum viable level %
        Drone::WhEVChargeRatePerTimeStep = 25000. / 3600;   // 25KW   rate of vehicle charge from drone(adjust for timeStep when simulation starts)
        Drone::WhDroneRechargePerTimeStep = 75000. / 3600;  // 75KW   rate of drone charge when parked(adjust for timeStep when simulation starts)
    }
    else {  // default "ehang184"
        Drone::droneChargeWh = 30000.0;                      // capacity of battery used to charge ev's, based on Ehang 184 load capacity - 200Kg
        Drone::droneFlyingWh = 14400.0;                      // capacity of battery used to power drone
        Drone::droneFlyingWhperTimeStep = Drone::droneFlyingWh / (23 * 60.);   // Ehang 184 has battery capacity of 14.4 KW giving 23 mins flight time
        Drone::droneChargeContingencyp = 0.05;              // minimum contingency level %
        Drone::droneChargeViablep = 0.3;                    // minimum viable level %
        Drone::WhEVChargeRatePerTimeStep = 25000. / 3600;   // 25KW   rate of vehicle charge from drone(adjust for timeStep when simulation starts)
        Drone::WhDroneRechargePerTimeStep = 75000. / 3600;  // 75KW   rate of drone charge when parked(adjust for timeStep when simulation starts)
    }
    Drone::droneMperSec = Drone::droneKMperh / 3.6;
    Drone::droneStepMperTimeStep = Drone::droneMperSec;                            // How far(metres) the drone will travel in one time step(adjust for timeStep when simulation starts)
    Drone::droneStepM2 = Drone::droneStepMperTimeStep * Drone::droneStepMperTimeStep;     // precompute - used in distance calculations(adjust for timeStep when simulation starts)
    Drone::minDroneCharge = Drone::droneChargeContingencyp * Drone::droneChargeWh;     // Thresholds to break off charging / flying
    Drone::minDroneFlyingWh = Drone::droneChargeContingencyp * Drone::droneFlyingWh;
    Drone::viableDroneCharge = Drone::droneChargeViablep * Drone::droneChargeWh;     // thresholds to allow allocation - ie enough charge to be useful
    Drone::viableDroneFlyingWh = Drone::droneChargeViablep * Drone::droneFlyingWh;
}

bool Drone::allocate(EV* ev, double requestedCharge) { // allocate this instance to an EV"""
    switch (myState) {
    case DroneState::CHARGINGDRONE:
    case DroneState::PARKED:
        dummyEVHide();
        break;
    default:
        break;
    }

    myEV = ev;
    TraCIPosition evpos = ev->getMyPosition();
    myRequestedCharge = requestedCharge;
    POI::setParameter(myID, "status", "allocated to " + ev->getID());
    if (GlobalFlags::myModelRendezvous)
        myState = DroneState::FLYINGTORENDEZVOUS;
    else
        myState = DroneState::FLYINGTOEV;
    return true;
}

void Drone::chargeMe() {   // Drone at hub - charge if needed"""
    if (myCharge < Drone::droneChargeWh) {
        myCharge += Drone::WhDroneRechargePerTimeStep;
        myChargeMeCount += 1;
    }
    if (myFlyingCharge < Drone::droneFlyingWh) {
        myFlyingCharge += Drone::WhDroneRechargePerTimeStep;
        myChargeMeFlyingCount += 1;
    }
    else if (myCharge >= Drone::droneChargeWh) { // fully charged so move to null state, avoiding calls to chargeMe
        // myCharge = Drone::droneChargeWh;
        // myFlyingCharge = Drone::droneFlyingWh;
        dummyEVHide();
        myState = DroneState::NULLSTATE;
        setViableCharge();
    }
}

void Drone::createDummyEV() {  // Create an EV type to use at charging stations whilst drone is charging"""
    if (Drone::dummyEVCreated)
        return;
    VehicleType::copy("DEFAULT_VEHTYPE", "Drone");
    VehicleType::setWidth("Drone", 0.1);
    VehicleType::setLength("Drone", 0.1);
    VehicleType::setMinGap("Drone", 0.0);
    VehicleType::setParameter("Drone", "has.battery.device", "True");
    VehicleType::setEmissionClass("Drone", "Energy/unknown");
    Drone::dummyEVCreated = true;
}

void Drone::dummyEVHide() {  // remove the dummy EVs""";
    if (GlobalFlags::ss->useChargeHubs and myDummyEVInserted) {
        string dummyFB = myID + "-FB";
        Vehicle::resume(dummyFB);
        Vehicle::remove(dummyFB);
        string dummyCB = myID + "-CB";
        Vehicle::resume(dummyCB);
        Vehicle::remove(dummyCB);
        myDummyEVInserted = false;
    }
}
    
void Drone::dummyEVInsert() {  // If we are generating charge station output add dummy EVs to the charge station for the drone batteries - whilst the drone is there"""
    if (GlobalFlags::ss->useChargeHubs) {
        string parkLane = myParkEP.edge + "_0";
        string dummyFB = myID + "-FB";
        Vehicle::add(dummyFB, myParkEP.edge, "Drone");
        Vehicle::setParameter(dummyFB, "device.battery.maximumBatteryCapacity", to_string(Drone::droneFlyingWh));
        Vehicle::setParameter(dummyFB, "device.battery.actualBatteryCapacity", to_string(myFlyingCharge));
        Vehicle::moveTo(dummyFB, parkLane, myParkEP.epos);
        Vehicle::setStop(dummyFB, myParkEP.edge, myParkEP.epos, 0, 10000.0, 1);

        string dummyCB = myID + "-CB";
        Vehicle::add(dummyCB, myParkEP.edge, "Drone");
        Vehicle::setParameter(dummyCB, "device.battery.maximumBatteryCapacity", to_string(Drone::droneChargeWh));
        Vehicle::setParameter(dummyCB, "device.battery.actualBatteryCapacity", to_string(myCharge));
        Vehicle::moveTo(dummyCB, parkLane, myParkEP.epos + 0.5);
        Vehicle::setStop(dummyCB, myParkEP.edge, myParkEP.epos + 0.5, 0, 10000.0, 1);
        myDummyEVInserted = true;
    }
}

/* move the drone along a straight line to pos by the amount Drone can move in a timeStep,
returns True if we've arrived at pos, False otherwise */
bool Drone::fly(libsumo::TraCIPosition pos) { 
    double ddy = pos.y - myPosition.y;
    double ddx = pos.x - myPosition.x;
 
    double x = 0.;
    double y = 0.;
    if ( ddx == 0 or ddy == 0 ) {
        if (ddx == 0) {
            if (ddy == 0) {
                x = 0.;
                y = 0.;
            }
            else {
                x = 0.;
                y = abs(ddy);
            }
        }
        else {  // ddy == 0
                x = abs(ddx);
                y = 0.0;
        }
    }
    else {
        x = abs(sqrt(droneStepM2 / (1.0 + pow(ddy/ddx, 2)) ));
        y = abs(x * ddy/ddx);
    }

    if (abs(ddx) <= x)  // will reach px in this step so set to ~exact distance
        x = pos.x + 0.001;
    else 
        {
        if (ddx > 0)
            x += myPosition.x;
        else
            x = myPosition.x - x;
        }

    if (abs(ddy) <= y) // will reach py in this step so set to ~exact distance
        y = pos.y + 0.001;
    else {
        if (ddy > 0)
            y += myPosition.y;
        else
            y = myPosition.y - y;
        }
    POI::setPosition(this->myID, x, y);
    myPosition.x = x; myPosition.y = y;

    if ((abs(x - pos.x) + abs(y - pos.y)) < 5.0) {   // we've arrived at px, py  - arbitrary 5m - two car kengths
        return true;
    }
    else
        return false;                    //we haven't got anywhere yet!
}

void Drone::logLine(std::string activity) { // Output discrete changes in charge levels for this drone"""
    string evID = "";
    string lane = "";
    double lanePos = 0;
    if (myEV != nullptr) {
        evID = myEV->getID();
        lane = Vehicle::getLaneID(evID);
        lanePos = Vehicle::getLanePosition(evID);
    }
    GlobalFlags::myDroneLog << setprecision(1) << GlobalFlags::ss->timeStep << "\t" << myID << "\t" << evID << "\t";
    GlobalFlags::myDroneLog << setprecision(4) << lane + "\t" << lanePos<< "\t" << myPosition.x << "\t" << myPosition.y << "\t";
    GlobalFlags::myDroneLog << setprecision(4) << myChargingWh << "\t" << myCharge << "\t" << myFlyingCharge << "\t" + activity << endl;
}

void Drone::notifyChase(bool chaseOK, int chaseSteps) { // from EV updating chases by this drone"""
    if (chaseOK) {
        myChaseCount += 1;
        myChaseSteps += chaseSteps;
    }
    else
        myBrokenChaseCount += 1;
}

void Drone::notifyEVFinished(EVState evState) { // EV tells us that it is charged or has left simulation so free self up"""
    switch (evState) {
    case EVState::LEFTSIMULATION:
        myBrokenEVCharges += 1;
        setMyParkPosition();
        if (myState == DroneState::CHARGINGDRONE or myState == DroneState::PARKED) {
            myEV = nullptr;
        }
        else
            park();
        break;

    case EVState::DRIVING:
        myFullCharges += 1;
        park();
        break;

    default:
        break;
    }
}

void Drone::park() {  //charge finished so park drone"""
    setMyParkPosition();
    myState = DroneState::FLYINGTOPARK;
    POI::setParameter(myID, "status", "Flying to park");
    GlobalFlags::cc->notifyDroneState(this);
    myEV = nullptr;
}

void Drone::parkingUpdate() {
    switch (myState) {
    case DroneState::FLYINGTOPARK:
    case DroneState::FLYINGTOCHARGE:
    case DroneState::PARKED:
    case DroneState::CHARGINGDRONE:
    case DroneState::NULLSTATE:
        break;
    default:
        myState = DroneState::FLYINGTOPARK;
        break;
    }
    update(myParkPosition);
}

void Drone::setMyParkPosition() {  // configure my parking/charging position"""
    if (! Drone::parkAtHome) 
    { // then we park at nearest hub
        hubLocation hLoc = GlobalFlags::ch->nearestHubLocation(myPosition).first;
        myParkPosition.x = hLoc.xypos.x;
        myParkPosition.y = hLoc.xypos.y;
        myParkEP = hLoc;
    }
}

void Drone::setViableCharge() {   // Check charge levels and see if we are viable - ie can be allocated"""
    if (myCharge >= Drone::viableDroneCharge and myFlyingCharge >= Drone::viableDroneFlyingWh)
        if (not myViableCharge) {
            myViableCharge = true;
            GlobalFlags::cc->notifyDroneState(this);  // cc only interested when we become viable
            POI::setColor(myID, { 0, 0, 255, 255 });
        }
    else
        myViableCharge = false;
}

void Drone::stepSecsAdjust(double stepSecs) {   // adjust timestep class variables for the actual timestep = no change when timeStep = 1sec"""
    Drone::droneStepMperTimeStep *= stepSecs;
    Drone::droneStepM2 = pow(Drone::droneStepMperTimeStep, 2);  // precompute - used in distance calculations
    Drone::droneFlyingWhperTimeStep *= stepSecs;
    Drone::WhEVChargeRatePerTimeStep *= stepSecs;
    Drone::WhDroneRechargePerTimeStep *= stepSecs;
}

pair<bool,double> Drone::update(libsumo::TraCIPosition pos) { // primary update - invoked directly when EV is managing drone"""
    bool updateStatus = true;
    double updatePower = 0.0;

    switch (myState) {
    case DroneState::PARKED:
        chargeMe();           // add charge upto limit because we can
        if (GlobalFlags::myDronePrint)
            logLine("Parked");
        break;

    case DroneState::FLYINGTORENDEZVOUS:
        if (usePower("fly")) {
            if (fly(pos)) {
                if (myEV != nullptr) { // usePower might have broken off after using power and set myEV to None
                    myState = DroneState::FLYINGTOEV;
                    if (GlobalFlags::myDronePrint)
                        logLine("Arrived at rendezvous");
                }
            }
            else {
                updateStatus = false;
                if (GlobalFlags::myDronePrint)
                    logLine("flying to rendezvous");
            }
        }
        else {
            fly(pos);
            if (GlobalFlags::myDronePrint)
                logLine("breaking off");
            updateStatus = false;
        }
        break;

    case DroneState::FLYINGTOEV:
        if (usePower("fly")) {
            if (fly(pos)) {
                if (myEV != nullptr) { // usePower might have broken off after using power and set myEV to None
                    POI::setParameter(myID, "status", "charging:" + myEV->getID());
                    myState = DroneState::CHARGINGEV;
                    if (GlobalFlags::myDronePrint)
                        logLine("arrived at ev");
                }
            }
            else {
                updateStatus = false;
                if (GlobalFlags::myDronePrint)
                    logLine("flying to ev");
            }
        }
        else {
            fly(pos);
            if (GlobalFlags::myDronePrint)
                logLine("breaking off");
            updateStatus = false;
        }
        break;

    case DroneState::CHARGINGEV:
        fly(pos);                          // 'fly' in this case is just moving with attached to the ev
        if (usePower("chargeEV"))          // False when charge broken off or completed
            updatePower = Drone::WhEVChargeRatePerTimeStep;
        else {
            updateStatus = false;
        }
        if (GlobalFlags::myDronePrint)
            logLine("charging EV");
        break;

    case DroneState::CHARGINGDRONE:
        chargeMe();
        if (GlobalFlags::myDronePrint)
            logLine("charging self");
        break;

    case DroneState::FLYINGTOCHARGE:
        usePower("");
        if (fly(myParkPosition)) {
            POI::setParameter(myID, "status", "parked - needs charge");
            POI::setColor(myID, { 0, 255, 0, 255 });
            myState = DroneState::CHARGINGDRONE;
            dummyEVInsert();
            if (GlobalFlags::myDronePrint)
                logLine("arrived at charge hub");
        }
        else {
            updateStatus = false;
            if (GlobalFlags::myDronePrint)
                logLine("flying to charge hub");
        }
        break;

    case DroneState::FLYINGTOPARK:     // note that previous version did not count power / distance used in arriving at hub
        usePower("fly");
        if (fly(myParkPosition)) {
            POI::setParameter(myID, "status", "Parked");
            myState = DroneState::PARKED;
            dummyEVInsert();
            if (GlobalFlags::myDronePrint)
                logLine("arrived at hub");
        }
        else {
            updateStatus = false;
            if (GlobalFlags::myDronePrint)
                logLine("flying to hub");
        }
        break;

    case DroneState::NULLSTATE:  // am charged so do nothing until allocated
        break;

    default:
        break;
    }
    return { updateStatus, updatePower };
}

bool Drone::usePower(std::string mode) {  // Am flying or charging an EV so adjust my charge levels"""
    bool breakOff = false;
    if (mode == "fly") {
        myFlyingCharge -= Drone::droneFlyingWhperTimeStep;
        myFlyingWh += Drone::droneFlyingWhperTimeStep;
        myFlyingCount += 1;
        if (myFlyingCharge < Drone::minDroneFlyingWh)
            breakOff = true;
    }
    else if (mode == "chargeEV") {
        myCharge -= Drone::WhEVChargeRatePerTimeStep;
        myChargingWh += Drone::WhEVChargeRatePerTimeStep;
        myRequestedCharge -= Drone::WhEVChargeRatePerTimeStep;
        if (myRequestedCharge <= 0) { //  we've charged the requested amount
            myEV->stopCharging(0);   // don't clear, full charge
            myFullCharges += 1;
            myRequestedCharge = 0;
            park();
            return false;  // ie we're not returning a charge
            }  
        if (myCharge < Drone::minDroneCharge)
            breakOff = true;
    }
    else {
        if (myState == DroneState::FLYINGTOCHARGE) {  // we're flying back to charge, only time we get here for now
            myFlyingCount += 1;
            myFlyingCharge -= Drone::droneFlyingWhperTimeStep;
        }
    }

    // problem - one of my batteries below contingency
    if (breakOff) {
        if (myEV != nullptr) {
            int request = int(myRequestedCharge + 1);
            myEV->stopCharging(request);  // tell EV how much charge we still need to apply to fulfil original request
            myEV = nullptr;
            myBrokenCharges += 1;
        }

        myRequestedCharge = 0;
        myViableCharge = false;
        setMyParkPosition();
        POI::setColor(myID, { 255, 0, 0, 255 });
        POI::setParameter(myID, "status", "Flying to charge");
        myState = DroneState::FLYINGTOCHARGE;

        GlobalFlags::cc->notifyDroneState(this);

        return false;
    }
    return true;
}

bool Drone::viable() {   // helper function to check whether we are able to be diverted"""
    switch (myState) {
    case DroneState::FLYINGTOCHARGE:
        return false;
    case DroneState::CHARGINGDRONE:
        return myViableCharge;
    default:
        return true;
    }
}