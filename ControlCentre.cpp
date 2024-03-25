#include <iostream>
#include <iomanip>
#include <chrono>
#include <string>
#include <tuple>
#include <vector>
#include <set>

#include "SumoDrone.h"
#include "ControlCentre.h"
#include "GlobalFlags.h"

#include "Drone.h"
#include "EV.h"
#include "ops.h"
#include <utility>

using namespace libsumo;
using namespace std;
/*
auto uCompare = [](urgency a, urgency b) {
    return (a.u < b.u);
    };*/

ControlCentre::ControlCentre(double pwEnergy, double pwUrgency, double pproximityRadius, int pmaxDrones) {
    wEnergy = pwEnergy;
    wUrgency = pwUrgency;
    proximityRadius = pproximityRadius;
    maxDrones = pmaxDrones;
    spawnedDrones = 0;
    insertedDummies = 0;
}

void ControlCentre::allocate(Drone* drone,  EV* ev) {
    allocatedEV[ev] = drone;
    allocatedDrone[drone] = ev;
    if (GlobalFlags::myModelRendezvous)
        ev->allocate(drone, findRendezvousXY(ev, drone));
    else {
        libsumo::TraCIPosition dummy;
        ev->allocate(drone, dummy);
    }
    drone->allocate(ev, requests[ev]);
    requests.erase(ev);
}


void ControlCentre::allocateDrones() {
    std::set<urgency, decltype(urgencyCmp)* > urgencyList = urgency::calcUrgency();

    size_t ld = freeDrones.size();                                    // available loaded drones
    int nd = ControlCentre::maxDrones - ControlCentre::spawnedDrones; // available new drones
    if (ld == 1)
        for (const auto& pr : urgencyList) {
            auto droneIt = freeDrones.begin();
            allocate(*droneIt, pr.ev);
            freeDrones.erase(*droneIt);
            break;
        }
    else if (ld < 1 && nd > 0)
        for (const auto& pr : urgencyList) {
            auto hub = GlobalFlags::ch->nearestHubLocation(pr.ev->getMyPosition());
            Drone* drone = new Drone(hub.first.xypos);
            spawnedDrones += 1;
            allocate(drone, pr.ev);
            if ((nd -= 1) <= 0)
                break;
        }
    else if (ld > 1)
        for (const auto& pr : urgencyList) { // need to find drone nearest the ev before we allocate
            TraCIPosition evPos = pr.ev->getMyPosition();
            double evDistance = numeric_limits<int>::max();
            Drone* nearestDrone = nullptr;
            for (auto& drone : freeDrones) {
                TraCIPosition dronePos = drone->getMyPosition();
                double distance = hypot(evPos.x - dronePos.x, evPos.y - dronePos.y);
                if (nearestDrone == nullptr) {
                    nearestDrone = drone;
                    evDistance = distance;
                }
                else if (distance < evDistance) {
                    nearestDrone = drone;
                    evDistance = distance;
                }
            }
            if (nearestDrone) {
                allocate(nearestDrone, pr.ev);
                freeDrones.erase(nearestDrone);
            }
            else if (nd > 0)   // no free drones so spawn another
                for (const auto& pr : urgencyList) {
                    TraCIPosition evPos = pr.ev->getMyPosition();
                    pair<hubLocation, double> hub = GlobalFlags::ch->nearestHubLocation(evPos);
                    Drone* drone = new Drone(hub.first.xypos);
                    ControlCentre::spawnedDrones += 1;
                    allocate(drone, pr.ev);
                    nd -= 1;
                    if (nd <= 0)
                        break;
                }
        }
    else  //   # ld == 0 nd > 0
        for (const auto& pr : urgencyList) {
            TraCIPosition evPos = pr.ev->getMyPosition();
            pair<hubLocation, double> hub = GlobalFlags::ch->nearestHubLocation(evPos);
            Drone* drone = new Drone(hub.first.xypos);
            ControlCentre::spawnedDrones += 1;
            allocate(drone, pr.ev);
            nd -= 1;
            if (nd <= 0)
                break;
        }
}



/* work out the edge and position of the EV, when it is deltaPos metres along the route from the current position
    to give us an approximation to the rendezvous position */
std::tuple<string, double, bool> ControlCentre::findEdgePos(string evID, double deltaPos) {
    int jnDelta = 150;  // no of travel metres 'lost' by vehicle crossing a junction, found by testing against random grid(ie allowance for vehicle slowing / accelerating)
    // find route and position of vehicle along the route - (this will always give us an edge)
    vector <string> evRoute = Vehicle::getRoute(evID);
    int idx = Vehicle::getRouteIndex(evID);
    string edge = evRoute[idx];
    double lanePosition = Vehicle::getLanePosition(evID);
    if (deltaPos < 0)
        deltaPos = 0; // print ("oops invalid call to findEdgePos:", deltaPos)

    // whilst we have the edge from above the vehicle could actually be on a junction which messes up the edge to edge calculation
    // so we get current road ID which can be a junction and if it is then skip along the route to the next edge
    string road = Vehicle::getRoadID(evID);
    if (road != edge) {       // ev is on a junction, set to start of next edge
        idx += 1;
        edge = evRoute[idx];   // what if idx is beyond end of route here?
        lanePosition = 0;
    }
    // 'travel' along edges on route until we've gone  deltaPos metres
    double newEVPosition = lanePosition + deltaPos;

    double laneLength = Lane::getLength(edge + "_0");
    while (newEVPosition > (laneLength + jnDelta)) {
        newEVPosition -= (laneLength + jnDelta);    // subtract jnDelta as the penalty in distance travelled, in the total travel time, for each junction
        idx += 1;
        if (idx >= evRoute.size())
        {// rv point is after end of route - so we can't charge
            return { evRoute[idx - 1], laneLength, false };   // set to end of route
        }
        edge = evRoute[idx];
        laneLength = Lane::getLength(edge + "_0");
    }
    newEVPosition = min(newEVPosition, laneLength);

    return { edge, newEVPosition, true };
}

    /* estimate a direct rendezvous point for the drone / vehicle - assumes constant vehicle speed
        apply a factor of 90 % to allow for acceleration / deceleration / % of time not at allowed speed
        algorithm from https ://www.codeproject.com/Articles/990452/Interception-of-Two-Moving-Objects-in-D-Space */
TraCIPosition ControlCentre::findRendezvousXY( EV* ev, Drone* drone) {
    string evID =  ev->getID();        // needed for Traci calls
    // assume speed on current edge is that for subsequent edges
    double evSpeed = 0.9 * Vehicle::getAllowedSpeed(evID);

    // work out how long it takes drone to fly to ev
    TraCIPosition posEV =  ev->getMyPosition();
    TraCIPosition posDrone = drone->getMyPosition();
    double distanceToEV = hypot(posDrone.x - posEV.x, posDrone.y - posEV.y);
    double crowFlies = distanceToEV / drone->myDt->droneMperSec;
    // how far vehicle can travel in same time
    double evCrowFlies = evSpeed * crowFlies;
    std::tuple<std::string, double, bool> edgePos = findEdgePos(evID, evCrowFlies);
    if (get<2>(edgePos)) {
        TraCIPosition posRV = Simulation::convert2D(get<0>(edgePos), get<1>(edgePos)); // get the x, y position after evCrowFlies metres
        // compute the velocity vector
        pair<double, double> evV = { (posRV.x - posEV.x) / crowFlies, (posRV.y - posEV.y) / crowFlies };

        pair <double, double> vectorFromEV = { posDrone.x - posEV.x, posDrone.y - posEV.y };
        double a = pow(drone->myDt->droneMperSec, 2) - pow(evSpeed, 2);
        double b = 2 * (vectorFromEV.first * evV.first + vectorFromEV.second * evV.second);
        double c = -pow(distanceToEV, 2);
        double bb4ac = (b * b) - (4 * a * c);
        double t1 = (-b + sqrt(bb4ac)) / (2 * a);
        double t2 = (-b - sqrt(bb4ac)) / (2 * a);

        if (t1 < 0 and t2 < 0)   // no solution we can't meet the EV
            return posDrone;

        double interceptDistance = 0.0;
        if (t1 > 0 and t2 > 0)  // both positive take the smallest
            interceptDistance = min(t1, t2) * evSpeed;
        else  // one is - ve so take the maximum
            interceptDistance = max(t1, t2) * evSpeed;

        edgePos = findEdgePos(evID, interceptDistance);
        if (get<2>(edgePos)) {      // will normally only fail if drone cannot reach EV under straight line intercept assumptions
            posRV = Simulation::convert2D(get<0>(edgePos), get<1>(edgePos));  // get the x, y position after evCrowFlies metres
            // Algorithm debug lines - show rendezvous point
            //string pid =  ev->getID() + " " + std::to_string(GlobalFlags::ss->timeStep);
            //POI::add(pid, posRV.x, posRV.y, { 255, 0, 255, 250 },"",0, "drone.png", 5.0, 5.0);
        }
        return posRV;   //this falls back to the original crow flies when straight line intercept fails
    }
    return posDrone;   // revert to direct intercept
}

void ControlCentre::notifyDroneState(Drone* drone) {  // Notification from Drone when charging finished or Drone has broken off the charge/flight"""
    if (allocatedDrone.find(drone) != allocatedDrone.end()) {
        allocatedEV.erase(allocatedDrone[drone]);
        allocatedDrone.erase(drone);
    }
    if (drone->viable()) {
        freeDrones.insert(drone);
        if (needChargeDrones.find(drone) != needChargeDrones.end())
            needChargeDrones.erase(drone);
    }
    else
        needChargeDrones.insert(drone);
}

void ControlCentre::notifyEVState(EV* ev, EVState evState, Drone* drone, double capacity) {   // Notification from EV - when EV has left simulation (or completed charge)"""
    double charge = 0.0;
    switch (evState) {
    case EVState::DRIVING:            // means charge complete  so calculate charge
    case EVState::CHARGEBROKENOFF:    // drone broke off charge so should have an entry in self.startChargeEV
        if (startChargeEV.find(ev) != startChargeEV.end()) {
            charge = (GlobalFlags::ss->timeStep - startChargeEV[ev]) * drone->myDt->WhEVChargeRatePerTimeStep;
            startChargeEV.erase(ev);
        }
        break;

    case EVState::CHARGEREQUESTED:   // just log request
        break;

    case EVState::CHARGINGFROMDRONE:      // charge started
        startChargeEV[ev] = GlobalFlags::ss->timeStep;
        break;

    case EVState::LEFTSIMULATION:
        if (startChargeEV.find(ev) != startChargeEV.end()) {
            if (startChargeEV[ev] > 0)
                charge = (GlobalFlags::ss->timeStep - startChargeEV[ev]) * drone->myDt->WhEVChargeRatePerTimeStep;
        }
        if (requests.find(ev) != requests.end()) {
            requests.erase(ev);
        }
        else if (allocatedEV.find(ev) != allocatedEV.end()) {
            allocatedDrone.erase(allocatedEV[ev]);
            allocatedEV.erase(ev);
        }
        break;

    case EVState::WAITINGFORRENDEZVOUS:   // don't see this yet
        break;

    case EVState::WAITINGFORDRONE:       // don't see this
        break;
    }

    if (GlobalFlags::myChargePrint) {
        string droneID = "-";
        if (drone != nullptr)
            droneID = drone->getID();
        GlobalFlags::myChargeLog << GlobalFlags::ss->timeStep << "\t" << ev->getID() << "\t" << evState << "\t" << droneID << "\t" << capacity << "\t" << charge << endl;
    }
}

bool droneCmp(const Drone* lurg, const Drone* rurg)
{
    return stoi(lurg->getID().substr(1)) < stoi(rurg->getID().substr(1));
}

void ControlCentre::printDroneStatistics(bool brief, string version, string runstring) {   //Print out Drone and EV statistics for the complete run"""
    // compute drone statistic totals
    int tmyFlyingCount = 0;           // used to compute distance travelled
    int tmyFullCharges = 0;           // count of complete charges
    int tmyBrokenCharges = 0;         // count of charges broken off - either by me out of charge
    int tmyBrokenEVCharges = 0;       // count of charges broken off - by EV leaving
    double tmyFlyingKWh = 0.0;           // KWh i've used flying
    double tmyChargingKWh = 0.0;         // KWh I've used charging EVs
    int tmyChargeMeFlyingCount = 0;   // wh i've charged my flying battery
    int tmyChargeMeCount = 0;         // wh i've used charging my EV charging battery
    double tmyResidualChargeKWh = 0.0;   // Wh left in charge battery at end
    double tmyResidualFlyingKWh = 0.0;   // Wh left in flying battery at end
    int tmyChaseCount = 0;            // no of successful chases(ie successfully got from rendezvous to EV)
    int tmyBrokenChaseCount = 0;      // no of broken chases(vehicle left after rendezvous but before drone got there)
    int tmyChaseSteps = 0;            // steps for succesful chases - used to compute average chase time

    set<Drone*, decltype(droneCmp)*> allDrones(droneCmp);
    merge(freeDrones.begin(), freeDrones.end(), needChargeDrones.begin(), needChargeDrones.end(),
        inserter(allDrones, allDrones.begin()));
    for (auto alloc : allocatedDrone)
        allDrones.insert(alloc.first);

    double tDroneDistance = 0.0;
    double tmyChargeMeFlyingKWh = 0.0;
    double tmyChargeMeKWh = 0.0;

    for (auto drone : allDrones) {
        tmyFlyingCount += drone->myFlyingCount;
        tmyFlyingKWh += (drone->myFlyingCount * drone->myDt->droneFlyingWhperTimeStep);

        tDroneDistance += (drone->myFlyingCount * drone->myDt->droneStepMperTimeStep);
 
        tmyFullCharges += drone->myFullCharges;
        tmyBrokenCharges += drone->myBrokenCharges;
        tmyBrokenEVCharges += drone->myBrokenEVCharges;
        
        tmyChargingKWh += (drone->myEVChargingCount * drone->myDt->WhEVChargeRatePerTimeStep);
        tmyChargeMeFlyingCount += drone->myChargeMeFlyingCount;
        tmyChargeMeFlyingKWh += (drone->myChargeMeFlyingCount * drone->myDt->WhDroneRechargePerTimeStep);
        tmyChargeMeCount += drone->myChargeMeCount;
        tmyChargeMeKWh += drone->myChargeMeCount * drone->myDt->WhDroneRechargePerTimeStep;
 
        tmyResidualFlyingKWh += drone->myFlyingCharge;
        tmyResidualChargeKWh += drone->myCharge;

        if (GlobalFlags::myModelRendezvous) {
            tmyChaseCount += drone->myChaseCount;
            tmyBrokenChaseCount += drone->myBrokenChaseCount;
            tmyChaseSteps += drone->myChaseSteps;
        }
    }

    tDroneDistance /= 1000.;
    tmyFlyingKWh /= 1000.;
    tmyChargingKWh /= 1000.;
    tmyChargeMeFlyingKWh /= 1000.;
    tmyChargeMeKWh /= 1000.;
    tmyResidualFlyingKWh /= 1000;
    tmyResidualChargeKWh /= 1000.;

    double averageChase = 0;
    if (GlobalFlags::myModelRendezvous) {
        // note chases + broken chases usually less than charge sessions total because some will break off before they get to rendezvous
        // ie chases are between rendezvous point and beginning charging - indicator of efficiency of rendezvous algorithm
        if (tmyChaseCount > 0)
            averageChase = (tmyChaseSteps * GlobalFlags::ss->stepSecs) / tmyChaseCount;
    }
    auto now = chrono::system_clock::now();
    auto itt = std::chrono::system_clock::to_time_t(now);
    std::ostringstream ss;
    ss << std::put_time(gmtime(&itt), "%FT%TZ");
    string timeStamp = ss.str();

    auto libsumoVersion = libsumo::Simulation::getVersion();

    // all done, dump the distance travelled by the drones and KW used
    cerr << "\n" << endl;      // new lines after the ...
    if (brief) {
        string sumoVersion = "(" +  libsumoVersion.first  + std::string(", ") + libsumoVersion.second + ")";

        cout << "Date\tRv\tOnce\tOutput\twE\twU\tradius\tSteps\tDrones"
            << "\tDistance\tFlyKWh\tchKWh\tFlyChgKWh\tChgKWh\trFlyKWh\trChKWh"
            << "\tEVs\tEVChg\tEVgap\tFull\tbrDrone\tbrEV\tChases\tAvg Chase\tBrk Chase" << endl;

        string flags = timeStamp;
        if (GlobalFlags::myModelRendezvous)
            flags += "\tTrue\t";
        else
            flags += "\tFalse\t";
        if (GlobalFlags::myOnlyChargeOnce)
            flags += "True\t";
        else
            flags += "False\t";
        if (GlobalFlags::myDronePrint)
            flags += "True\t";
        else
            flags += "False\t";
        cout << std::fixed << std::setprecision(2);

        cout << flags << wEnergy << "\t" << wUrgency << "\t" << proximityRadius << "\t" << GlobalFlags::ss->timeStep << "\t"
            << spawnedDrones << "\t" << tDroneDistance << "\t" << tmyFlyingKWh << "\t" << tmyChargingKWh << "\t"
            << tmyChargeMeFlyingKWh << "\t" << tmyChargeMeKWh << "\t" << tmyResidualFlyingKWh << "\t" << tmyResidualChargeKWh << "\t"
            << EV::evCount << "\t" << EV::evChargeSteps * Drone::d0Type->WhEVChargeRatePerTimeStep / 1000. << "\t" << EV::evChargeGap / (1000. * EV::evCount) << "\t"
            << tmyFullCharges << "\t" << tmyBrokenCharges << "\t" << tmyBrokenEVCharges << "\t";
        if (GlobalFlags::myModelRendezvous)
            cout << tmyChaseCount << "\t" << averageChase << "\t" << tmyBrokenChaseCount << "\t" << runstring  << "\t" << version << "\t" << sumoVersion << endl;
        else
            cout << "\t\t\t" << runstring << "\t" << version << "\t" << sumoVersion << endl;

    }
    else {
        string flags("\tModel flags:\tRendezvous: ");
        if (GlobalFlags::myModelRendezvous)
            flags += "True\tCharge Once: ";
        else
            flags += "False\tCharge Once: ";
        if (GlobalFlags::myOnlyChargeOnce)
            flags += "True\tDrone print: ";
        else
            flags += "False\tDrone print: ";
        if (GlobalFlags::myDronePrint)
            flags += "True\n";
        else
            flags += "False\n";

        cout << std::fixed << std::setprecision(1);
        cout << "\nSummary Statistics:\t\t" << timeStamp << flags;
        cout << "\tEnergy Weight: " << wEnergy << "\tUrgency Weight: " << wUrgency;
        cout << "\tProximity radius(m): " << proximityRadius << "\tTime steps : " << GlobalFlags::ss->timeStep << endl;
        cout << "\n\tDrone Totals: (" << spawnedDrones << ")" << endl;
        cout <<  std::setprecision(2);
        cout << "\t\tDistance Km: \t" << tDroneDistance << "\n\t\tFlying KWh: \t" << tmyFlyingKWh << "\n\t\tCharging KWh: \t" << tmyChargingKWh << endl;
        cout << "\tDrone Charger usage:\n\t\tFlying KWh:\t" << tmyChargeMeFlyingKWh << "\n\t\tCharge KWh: \t" << tmyChargeMeKWh << endl;
        cout << "\tResiduals:\n\t\tFlying KWh:\t" << tmyResidualFlyingKWh << "\n\t\tCharging KWh: \t" << tmyResidualChargeKWh << endl;
        cout << std::setprecision(1);
        cout << "\n\tEV Totals: (" << EV::evCount << ")\n\t\tCharge KWh:\t" << EV::evChargeSteps * Drone::d0Type->WhEVChargeRatePerTimeStep / 1000. << endl;
        cout << "\t\tCharge Gap KWh: " << EV::evChargeGap / (1000. * EV::evCount) << endl;
        cout << "\t\tCharge Sessions:\n\t\t\tFull charges:\t" << tmyFullCharges << "\n\t\t\tPart (drone):\t" << tmyBrokenCharges << "\n\t\t\tPart (ev) :\t" << tmyBrokenEVCharges << endl;

        if (GlobalFlags::myModelRendezvous)
            cout << "\n\tSuccessful chases: " << tmyChaseCount << "\tAverage chase time: " << averageChase << "s\tbroken Chases: " << tmyBrokenChaseCount << endl;

        cout << "\nDiscrete Drone data:" << endl;
        cout << std::setprecision(2);
        for (auto drone : allDrones) {
            double droneDistance = drone->myFlyingCount * drone->myDt->droneStepMperTimeStep / 1000.;
            double droneFlyingKWh = drone->myFlyingCount * drone->myDt->droneFlyingWhperTimeStep / 1000.;
            double droneChargeKWh = drone->myEVChargingCount * drone->myDt->WhEVChargeRatePerTimeStep / 1000.;
            cout << "\tdrone: " << drone->getID() << "\tKm: " << droneDistance << "\tCharge KW: " << droneChargeKWh;
            cout << "\tFlyingKW: " << droneFlyingKWh << "\tResidual (chargeWh: " << drone->myCharge << "\tflyingWh: " << drone->myFlyingCharge << ")" << endl;
        }
    }
}

void ControlCentre::requestCharge(EV* ev, double capacity, double requestedWh = 2000.0) {
    requests[ev] = requestedWh;
    if (GlobalFlags::myChargePrint)
        GlobalFlags::myChargeLog << GlobalFlags::ss->timeStep << "\t" <<  ev->getID() << "\t" << "CHARGEREQUESTED" << "\t" << "" << "\t" << capacity << "\t" << 0.0 << "\t" << requestedWh << endl;
 }

void ControlCentre::tidyDrones() {
    if (insertedDummies > 0) {
        set<Drone*, decltype(droneCmp)*> dummyDrones(droneCmp);
        merge(freeDrones.begin(), freeDrones.end(), needChargeDrones.begin(), needChargeDrones.end(),
            inserter(dummyDrones, dummyDrones.begin()));

        for (auto drone : dummyDrones)
            if (drone->myDummyEVInserted)
                drone->dummyEVHide();
    }
}

void ControlCentre::update() { //Management of 'control centre' executed by simulation on every step
    size_t availableDrones = freeDrones.size() + maxDrones - spawnedDrones;
   if (availableDrones > 0 and requests.size() > 0)
        allocateDrones();
   
    // Control centre manages parking / charging of drones
    // each EV 'manages' the drone allocated to them

    for (Drone* drone : freeDrones) {
        drone->parkingUpdate();
       }
    for (Drone* drone : needChargeDrones) {
        drone->parkingUpdate();
    }
}

