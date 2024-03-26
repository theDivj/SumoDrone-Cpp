#include <string>
#include <libsumo/libsumo.h>
#include "urgency.h"
#include "ControlCentre.h"

using namespace std;
using namespace libsumo;

bool urgencyCmp(const urgency* lurg, const urgency* rurg){
    return lurg->urg < rurg->urg;
};

void urgencyListOut(string txt, std::set <urgency*, decltype(urgencyCmp)* >& uList) {
    cout << txt << "\n";
    for (auto& urgnc : uList) {
        cout << "\t" << urgnc->urg <<"\n";
    }
    cout << endl;
}
/* urgency defined as distance to nearest hub / distance ev can travel on charge.
        creates a list of ev's that want charge, have not been allocated a drone
        */
std::set<urgency*, decltype(urgencyCmp)* > urgency::calcUrgency() {
    std::set <urgency*, decltype(urgencyCmp)* > urgencyList(urgencyCmp);
    if (ControlCentre::requests.size() == 1)
        for (auto ev : ControlCentre::requests)
        {
            auto ret = urgencyList.insert(new urgency(1.0, ev.first, ev.second));
        }
    else {
        bool firstCall = true;
        for (auto ev : ControlCentre::requests) {
            // note drivingDistance can be very large - float max if there is no hub on the remaining route
            string evID = ev.first->getID();
            if (firstCall)
                ev.first->setMyPosition();
            TraCIPosition evPos = ev.first->getMyPosition();

            pair<hubLocation, double> hDist = GlobalFlags::ch->findNearestHub(evPos);
            //# hub, hubDistance = GG.ch.findNearestHubDriving(evID)

            double evRange = 200.0; 
            if (ControlCentre::wUrgency > 0.0) {   // if we have an ugency weight then we need to calculate the range
                double distance = Vehicle::getDistance(evID);
                if (distance > 10000) {   // can compute real range after we've been driving for a while - arbitrary 10km
                    double mWh = distance / stod(Vehicle::getParameter(evID, "device.battery.totalEnergyConsumed"));
                    evRange = stod(Vehicle::getParameter(evID, "device.battery.actualBatteryCapacity")) * mWh / 1000.;
                }
                else   // otherwise just a guesstimate
                    evRange = stod(Vehicle::getParameter(evID, "device.battery.actualBatteryCapacity")) * ev.first->getMyKmPerWh();
            }
            double proximity = 1.0;
            double urgencyv = 1.0;
            if (ControlCentre::wEnergy != 0.0) {       // We have a weight so need to calculate proximity
                pair<unordered_set<EV*>, double> neighbours = getNeighboursNeedingCharge(ev.first, firstCall);
                firstCall = false;
                // find distance for nearest drone to this eV - usually only one drone so will be the one allocated
                double droneDist = ControlCentre::proximityRadius;   // default - should never happen - otherwise fn wouldn't be called
                if (ControlCentre::freeDrones.size() > 0) {
                    droneDist = numeric_limits<double>::max();
                    for (auto drone : ControlCentre::freeDrones) {
                        TraCIPosition dPos = drone->getMyPosition();
                        double dist = std::hypot(evPos.x - dPos.x, evPos.y - dPos.y);
                        if (dist < droneDist)
                            droneDist = dist;
                    }
                }

                // proximity factors - smallest value is most important = 'nearest
                if (neighbours.first.size() > 1)
                    neighbours.second /= neighbours.first.size();   // set distance  'smaller' the more neighbours there are
                //delete neighbours

            // add in drone distance - ie smallest proximity will have closest drone
                if (hDist.second == numeric_limits<double>::max())
                    proximity = droneDist + neighbours.second;     //  + evRange
                else
                    proximity = droneDist + neighbours.second;     //   /drivingDistance
            }
            else
                urgencyv = hDist.second / evRange;

            double CEC = (proximity * ControlCentre::wEnergy) + (urgencyv * ControlCentre::wUrgency);
            auto ret = urgencyList.insert(new urgency(CEC, ev.first, ev.second));
        }
    }
    //urgencyListOut("", urgencyList);
    return urgencyList;
}

/* find all the ev's that are requesting a charge and compute the mean distance to these
        note calling math.dist which will use sqrt is actually faster than comparing distances to the square
        we only update the ev positions on first call because we will repeat call to this fn for each ev in creating urgency list */
std::pair<std::unordered_set<EV*>, double> urgency::getNeighboursNeedingCharge(EV* ev, bool firstCall) {
    std::unordered_set<EV*> neighbours;
    double meanDist = 0.0;
    if (firstCall)
        ev->setMyPosition();
    TraCIPosition evPos = ev->getMyPosition();

    for (auto nEV : ControlCentre::requests) {
        if (nEV.first->getID() == ev->getID())
            continue;

        if (firstCall)
            nEV.first->setMyPosition();
        TraCIPosition vPos = nEV.first->getMyPosition();

        double xdist = hypot(evPos.x - vPos.x, evPos.y - vPos.y);
        if (xdist < ControlCentre::proximityRadius) {
            neighbours.emplace(nEV.first);
            meanDist += xdist;
        }
    }
    if (meanDist > 0.0)      // we have at least 1 ev so calculate the actual mean distance
        meanDist = meanDist / neighbours.size();

    return pair<unordered_set<EV*>, double> {neighbours, meanDist};
}

std::ostream& urgency::operator<<(std::ostream& os) {
    os << urg << "\t" << ev->getID() << "\t" << requestedCharge; return os;
}
