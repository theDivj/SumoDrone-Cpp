#include "Drone.h"
#include "EV.h"
#include <string>
#include "GlobalFlags.h"
#include <cmath>
#include <exception>
#include <iomanip>
#include <vector>
#include <iostream>
#include "DroneType.h"

using namespace libsumo;
using namespace std;

// initialise class variables
int Drone::droneIDCount = 0;
bool Drone::parkAtHome = false;
bool Drone::dummyEVCreated = false;
DroneType* Drone::d0Type = new DroneType;

bool Drone::printedType = false;

Drone::Drone(TraCIPosition pos, string poi, DroneType* DT) {
    if (poi.empty()) {    // drones created from defaults
        Drone::droneIDCount += 1;
        myDt = d0Type;
        myID = "d" + to_string(Drone::droneIDCount);
    }
    else {                           // drones being created when zeroDrone is true - ie from POIs
        Drone::droneIDCount += 1;    // just to keep a count of drones - not to be used to create name
        myID = poi;
        myDt = DT;
    }

    myDt->droneKMperh = GlobalFlags::myDroneKmPerHr;
    myDt->setDerived(GlobalFlags::ss->getStepSecs());

    myPosition = pos;
    myParkPosition = myPosition;
    //myParkEP;
    myCharge = myDt->droneChargeWh;              
    myFlyingCharge = myDt->droneFlyingWh;
    myViableCharge = true;
    myState = DroneState::NULLSTATE;
    myEV = nullptr;

    // logging variables
    myFlyingCount = 0;            // used to compute distance travelled
    myFullCharges = 0;             // count of complete charges
    myBrokenCharges = 0;           // count of charges broken off - by me out of charge
    myBrokenEVCharges = 0;         // count of charges broken off - by EV(leaving)
    myEVChargingCount = 0;            // Count of timesteps charging EVs
    myChargeMeFlyingCount = 0;   // wh i've charged my flying battery
    myChargeMeCount = 0;         // wh i've used charging my EV charging battery
    myChaseCount = 0;              // count of complete chases - ie got from rendezvous to ev
    myBrokenChaseCount = 0;        // count of broken chases where we didn't get to ev before it left sim
    myChaseSteps = 0;              // count of steps in all complete chases - used with myChaseCount to compute average
    myRequestedCharge = 0;         // the amount of charge requested by the EV
    myDummyEVInserted = false;     // whether the dummy EVs have been inserted
        // finally create the POI representing our drone  - unless its alreday there from the add file(s)
    if ( poi != myID )
        POI::add(myID, pos.x, pos.y, myDt->droneColour, "", 250, myDt->droneImageFile, myDt->droneWidth, myDt->droneHeight);
    else {
        POI::setColor(myID, myDt->droneColour);
        POI::setImageFile(myID, myDt->droneImageFile);
        POI::setWidth(myID, myDt->droneWidth);
        POI::setHeight(myID, myDt->droneHeight);
    }

    if (GlobalFlags::ss->getUseChargeHubs() and not dummyEVCreated)  
        createDummyEV();
    //if (not Drone::printedType) {
     //   cerr << *myDt << endl;
     //   printedType = true;
    //}
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//  static methods

void Drone::setDroneType(bool useOneBattery, std::string droneType="ehang184") {  // Support different drone definitions - initially to give us a drone that doesn't need charging"""

    d0Type->droneKMperh = GlobalFlags::getDroneSpeed(); // get speed override if any

    // EV charging battery size is constrained by drone carrying capacity * average battery energy density(currently ~150Wh / Kg)
    if (droneType == "ehang184x") {  // ehang 184 with artificially increased battery sizes so they don't need recharging
        d0Type->droneChargeWh = 3000000.0;    // 100 * actual
        d0Type->droneFlyingWh = 14400000.0;
        d0Type->droneFlyingWhperTimeStep = 14400.0 / (23 * 60.);
        d0Type->droneChargeContingencyp = 0.05;              // minimum contingency level %
        d0Type->droneChargeViablep = 0.3;                    // minimum viable level %
        d0Type->WhEVChargeRatePerTimeStep = 25000 / 3600.;   // 25KW   rate of vehicle charge from drone(adjust for timeStep when simulation starts)
        d0Type->WhDroneRechargePerTimeStep = 75000 / 3600.;  // 75KW   rate of drone charge when parked(adjust for timeStep when simulation starts)
        d0Type->useOneBattery = useOneBattery;               // if true use the charge battery to fly and charge
    }
    else {  // default "ehang184"
        d0Type->droneChargeWh = 30000.0;                      // capacity of battery used to charge ev's, based on Ehang 184 load capacity - 200Kg
        d0Type->droneFlyingWh = 14400.0;                      // capacity of battery used to power drone
        d0Type->droneFlyingWhperTimeStep = d0Type->droneFlyingWh / (23 * 60.);   // Ehang 184 has battery capacity of 14.4 KW giving 23 mins flight time
        d0Type->droneChargeContingencyp = 0.05;              // minimum contingency level %
        d0Type->droneChargeViablep = 0.3;                    // minimum viable level %
        d0Type->WhEVChargeRatePerTimeStep = 25000 / 3600.;   // 25KW   rate of vehicle charge from drone(adjust for timeStep when simulation starts)
        d0Type->WhDroneRechargePerTimeStep = 75000 / 3600.;  // 75KW   rate of drone charge when parked(adjust for timeStep when simulation starts)
        d0Type->useOneBattery = useOneBattery;               // if true use the charge battery to fly and charge
    }
}

//retrieve drone parameters from Sumo POI d0 - returns true if parameters set otherwise false
int Drone::setDroneTypeFromPOI(bool useOneBattery, bool zeroDrone) {
    // first set the defaults from d0 if it exists
    vector<string> POIlist = POI::getIDList();
    if (POIlist.size() > 0)
        for (string poi : POIlist) {
            if (poi == "d0") {
                double dWidth = POI::getWidth(poi);
                if (dWidth > 1.) Drone::d0Type->droneWidth = dWidth;
                double dHeight = POI::getHeight(poi);
                if (dHeight > 1.) Drone::d0Type->droneHeight = dHeight;
                TraCIColor dColor = POI::getColor(poi);
                if (dColor.getString().size() > 1) Drone::d0Type->droneColour = dColor;
                string dImageFile = POI::getImageFile(poi);
                if (dImageFile.size() > 1) Drone::d0Type->droneImageFile = dImageFile;

                string dDroneKMperh = POI::getParameter(poi, "droneKMperh");
                if (dDroneKMperh.size() > 1) Drone::d0Type->droneKMperh = stod(dDroneKMperh);

                string dDroneChargeWh = POI::getParameter(poi, "droneChargeWh");
                if (dDroneChargeWh.size() > 1) Drone::d0Type->droneChargeWh = stod(dDroneChargeWh);

                string dDroneFlyingWh = POI::getParameter(poi, "droneFlyingWh");
                if (dDroneFlyingWh.size() > 1) Drone::d0Type->droneFlyingWh = stod(dDroneFlyingWh);

                string dDroneFlyingMinutes = POI::getParameter(poi, "droneFlyingMinutes");
                if (dDroneFlyingMinutes.size() > 1) Drone::d0Type->droneFlyingWhperTimeStep = Drone::d0Type->droneFlyingWh/(60. * stoi(dDroneFlyingMinutes));

                string dDroneChargeContingencyp = POI::getParameter(poi, "droneChargeContingencyp");
                if (dDroneChargeContingencyp.size() > 1) Drone::d0Type->droneChargeContingencyp = stod(dDroneChargeContingencyp);

                string dDroneChargeViablep = POI::getParameter(poi, "droneChargeViablep");
                if (dDroneChargeViablep.size() > 1) Drone::d0Type->droneChargeViablep = stod(dDroneChargeViablep);

                string dWhEVChargeRate = POI::getParameter(poi, "WhEVChargeRate");
                if (dWhEVChargeRate.size() > 1) Drone::d0Type->WhEVChargeRatePerTimeStep = stoi(dWhEVChargeRate)/3600.;

                string dWhDroneRechargeRate = POI::getParameter(poi, "WhDroneRechargeRate");
                if (dWhDroneRechargeRate.size() > 1) Drone::d0Type->WhDroneRechargePerTimeStep = stoi(dWhDroneRechargeRate)/3600.;

                string duseOneBattery = POI::getParameter(poi, "useOneBattery");
                if (duseOneBattery.size() > 0) Drone::d0Type->useOneBattery = true;
                else
                    Drone::d0Type->useOneBattery = useOneBattery;

                POI::remove(poi);
                if (not zeroDrone){
                    return 1;  // ie we've set the d0Type so can return
                }
            }
        }
    // if zeroDrone is set then initialise from any drone definitions in simulation
    //  retrieve the list again because we should have removed d0
    int poiDroneCount = 0;
    POIlist = POI::getIDList(); 
    if (zeroDrone and (POIlist.size() > 0)) {
        for (string poi : POIlist) {
            if (POI::getType(poi) == "drone") {   // weve removed the d0 drone
                DroneType* DT = new DroneType(*d0Type);
                //*DT = *d0Type;
                double dWidth = POI::getWidth(poi);
                if (dWidth > 1.) DT->droneWidth = dWidth;
                double dHeight = POI::getHeight(poi);
                if (dHeight > 1.) DT->droneHeight = dHeight;
                TraCIColor dColor = POI::getColor(poi);
                if (dColor.getString().size() > 1) DT->droneColour = dColor;
                string dImageFile = POI::getImageFile(poi);
                if (dImageFile.size() > 1) DT->droneImageFile = dImageFile;

                string dDroneKMperh = POI::getParameter(poi, "droneKMperh");
                if (dDroneKMperh.size() > 1) DT->droneKMperh = stod(dDroneKMperh);

                string dDroneChargeWh = POI::getParameter(poi, "droneChargeWh");
                if (dDroneChargeWh.size() > 1) DT->droneChargeWh = stod(dDroneChargeWh);

                string dDroneFlyingWh = POI::getParameter(poi, "droneFlyingWh");
                if (dDroneFlyingWh.size() > 1) DT->droneFlyingWh = stod(dDroneFlyingWh);

                string dDroneFlyingMinutes = POI::getParameter(poi, "droneFlyingMinutes");
                if (dDroneFlyingMinutes.size() > 1) DT->droneFlyingWhperTimeStep = DT->droneFlyingWh / (60. * stoi(dDroneFlyingMinutes));

                string dDroneChargeContingencyp = POI::getParameter(poi, "droneChargeContingencyp");
                if (dDroneChargeContingencyp.size() > 1) DT->droneChargeContingencyp = stod(dDroneChargeContingencyp);

                string dDroneChargeViablep = POI::getParameter(poi, "droneChargeViablep");
                if (dDroneChargeViablep.size() > 1) DT->droneChargeViablep = stod(dDroneChargeViablep);

                string dWhEVChargeRate = POI::getParameter(poi, "WhEVChargeRate");
                if (dWhEVChargeRate.size() > 1) DT->WhEVChargeRatePerTimeStep = stoi(dWhEVChargeRate) / 3600.;

                string dWhDroneRechargeRate = POI::getParameter(poi, "WhDroneRechargeRate");
                if (dWhDroneRechargeRate.size() > 1) DT->WhDroneRechargePerTimeStep = stoi(dWhDroneRechargeRate) / 3600.;

                string duseOneBattery = POI::getParameter(poi, "useOneBattery");
                if (duseOneBattery.size() > 0) Drone::d0Type->useOneBattery = true;

                // create the drone
                try {
                    TraCIPosition pos = POI::getPosition(poi);
                    GlobalFlags::cc->freeDrones.insert(new Drone(pos, poi, DT));
                    poiDroneCount += 1;
                }
                catch (const std::exception& err) { cerr << "Drone " + poi + " creation failed. :-" << err.what() << endl; }
            }
        }
        return poiDroneCount;
    }
    return 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//  members
//
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
    myRequestedCharge = requestedCharge;
    POI::setParameter(myID, "status", "allocated to " + ev->getID());
    if (GlobalFlags::myModelRendezvous)
        myState = DroneState::FLYINGTORENDEZVOUS;
    else
        myState = DroneState::FLYINGTOEV;
    return true;
}

void Drone::chargeMe() {   // Drone at hub - charge if needed"""
    if (myCharge < myDt->droneChargeWh) {
        myCharge += myDt->WhDroneRechargePerTimeStep;
        myChargeMeCount += 1;
    }
    if (myFlyingCharge < myDt->droneFlyingWh) {
        myFlyingCharge += myDt->WhDroneRechargePerTimeStep;
        myChargeMeFlyingCount += 1;
    }
    else if (myCharge >= myDt->droneChargeWh) { // fully charged so move to null state, avoiding calls to chargeMe
        // myCharge = myDt->droneChargeWh;
        // myFlyingCharge = myDt->droneFlyingWh;
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
    if (GlobalFlags::ss->getUseChargeHubs() and myDummyEVInserted) {
        string dummyFB = myID + "-FB";
        try {
            int stState = Vehicle::getStopState(dummyFB);  // only need to resume if we actually parked
            if (stState && 2 == 2)
                Vehicle::resume(dummyFB);
        }
        catch (const std::exception& err) {}
        Vehicle::remove(dummyFB);
        GlobalFlags::cc->insertedDummies -= 1;
        
        string dummyCB = myID + "-CB";
        try {
            int stState = Vehicle::getStopState(dummyCB);
            if (stState && 2 == 2)
                Vehicle::resume(dummyCB);
        }
        catch (const std::exception& err) {}
        Vehicle::remove(dummyCB);
        GlobalFlags::cc->insertedDummies -= 1;
 
        myDummyEVInserted = false;
    }
}
    
void Drone::dummyEVInsert() {  // If we are generating charge station output add dummy EVs to the charge station for the drone batteries - whilst the drone is there"""
    if (GlobalFlags::ss->getUseChargeHubs()) {
        string dummyFB = myID + "-FB";
        Vehicle::add(dummyFB, myParkEP.edge, "Drone", "now", "0", to_string(myParkEP.epos));
        Vehicle::setParameter(dummyFB, "device.battery.maximumBatteryCapacity", to_string(myDt->droneFlyingWh));
        Vehicle::setParameter(dummyFB, "device.battery.actualBatteryCapacity", to_string(myFlyingCharge));
        Vehicle::setStop(dummyFB, myParkEP.edge, myParkEP.epos, 0, 10000.0, 1);
        GlobalFlags::cc->insertedDummies += 1;

        string dummyCB = myID + "-CB";
        Vehicle::add(dummyCB, myParkEP.edge, "Drone", "now", "0", to_string(myParkEP.epos + 0.5));
        Vehicle::setParameter(dummyCB, "device.battery.maximumBatteryCapacity", to_string(myDt->droneChargeWh));
        Vehicle::setParameter(dummyCB, "device.battery.actualBatteryCapacity", to_string(myCharge));
        Vehicle::setStop(dummyCB, myParkEP.edge, myParkEP.epos + 0.5, 0, 10000.0, 1);
        GlobalFlags::cc->insertedDummies += 1;

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
        x = abs(sqrt(myDt->droneStepM2 / (1.0 + pow(ddy/ddx, 2)) ));
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

void Drone::logLine(std::string activity) const { // Output discrete changes in charge levels for this drone"""
    string evID = "";
    string lane = "";
    double lanePos = 0;
    if (myEV != nullptr) {
        evID = myEV->getID();
        lane = Vehicle::getLaneID(evID);
        lanePos = Vehicle::getLanePosition(evID);
    }
    GlobalFlags::myDroneLog << setprecision(1) << GlobalFlags::ss->getTimeStep() << "\t" << myID << "\t" << evID << "\t";
    GlobalFlags::myDroneLog << setprecision(4) << lane + "\t" << lanePos<< "\t" << myPosition.x << "\t" << myPosition.y << "\t";
    GlobalFlags::myDroneLog << setprecision(4) << myEVChargingCount * myDt->WhEVChargeRatePerTimeStep << "\t" << myCharge << "\t" << myFlyingCharge << "\t" + activity << endl;
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
        if ( myState == DroneState::CHARGINGEV)
            myBrokenEVCharges += 1;
        setMyParkPosition();
        if (myState == DroneState::CHARGINGDRONE or myState == DroneState::PARKED) {
            dummyEVHide();
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
    if (myCharge >= myDt->viableDroneCharge and myFlyingCharge >= myDt->viableDroneFlyingWh)
        if (not myViableCharge) {
            myViableCharge = true;
            GlobalFlags::cc->notifyDroneState(this);  // cc only interested when we become viable
            POI::setColor(myID, myDt->droneColour);
        }
    else
        myViableCharge = false;
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
            updatePower = myDt->WhEVChargeRatePerTimeStep;
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
        myFlyingCount += 1;
        if (myDt->useOneBattery) {
            myCharge -= myDt->droneFlyingWhperTimeStep;
            if (myCharge < myDt->minDroneCharge)
                breakOff = true;
        }
        else {
            myFlyingCharge -= myDt->droneFlyingWhperTimeStep;
            if (myFlyingCharge < myDt->minDroneFlyingWh)
                breakOff = true;
        }
    }
    else if (mode == "chargeEV") {
        myCharge -= myDt->WhEVChargeRatePerTimeStep;
        myEVChargingCount += 1;
        myRequestedCharge -= myDt->WhEVChargeRatePerTimeStep;
        if (myRequestedCharge <= 0) { //  we've charged the requested amount
            myEV->stopCharging(0);   // don't clear, full charge
            myFullCharges += 1;
            myRequestedCharge = 0;
            park();
            return false;  // ie we're not returning a charge
            }  
        if (myCharge < myDt->minDroneCharge)
            breakOff = true;
    }
    else {
        if (myState == DroneState::FLYINGTOCHARGE) {  // we're flying back to charge, only time we get here for now
            myFlyingCount += 1;
            if (myDt->useOneBattery)
                myCharge -= myDt->droneFlyingWhperTimeStep;
            else    
                myFlyingCharge -= myDt->droneFlyingWhperTimeStep;
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

bool Drone::viable() const {   // helper function to check whether we are able to be diverted"""
    switch (myState) {
    case DroneState::FLYINGTOCHARGE:
        return false;
    case DroneState::CHARGINGDRONE:
        return myViableCharge;
    default:
        return true;
    }
}