#include <string>
#include <tuple>
#include <libsumo/libsumo.h>
#include "EvAllocationFactors.h"

#include "ControlCentre.h"

using namespace std;
using namespace libsumo;

bool EvAllocationFactorsCmp(const EvAllocationFactors* lurg, const EvAllocationFactors* rurg) {
    return lurg->CEC < rurg->CEC;
};

void EvAllocationFactorsListOut(string txt, std::set <EvAllocationFactors*, decltype(EvAllocationFactorsCmp)* >& uList) {
    cout << txt << "\n\t";
    for (auto& urgnc : uList) {
        cout  << urgnc->getID() << " " << urgnc->CEC << " " << urgnc->urgencyPosition << ",  ";
    }
    cout << endl;
}
/* urgency defined as distance to nearest hub / distance ev can travel on charge.
   urgencyPosition is the ranking, starting at zero of EV urgencies - most urgent at 0, next at 1 
   proximity is related to the number of EVs needing a charge within a radius from each EV
     lowest value of proximity reflects an EV with the most, nearest neighbours
        The allocation factors comprise urgency, proximity

        fn creates a list of ev's that want charge, have not been allocated a drone
           ordered by these factors weighted by the runstring weights
        */
std::set<EvAllocationFactors*, decltype(EvAllocationFactorsCmp)* > EvAllocationFactors::calcEvAllocationFactors() {
    std::set <EvAllocationFactors*, decltype(EvAllocationFactorsCmp)* > EvAllocationFactorsList(EvAllocationFactorsCmp);

    double maxProximity = 0.0;
    double maxUrgency = 0.0;

    if (ControlCentre::requests.size() == 1)
        for (auto ev : ControlCentre::requests)
        {
            auto ret = EvAllocationFactorsList.insert(new EvAllocationFactors(0.0, 0.0, 0.0, ev.first, ev.second, 0));
        }
    else {
        bool firstCall = true;
        for (auto ev : ControlCentre::requests) {
            // note drivingDistance can be very large - float max if there is no hub on the remaining route
            string evID = ev.first->getID();
            if (firstCall)
                ev.first->setMyPosition();
            TraCIPosition evPos = ev.first->getMyPosition();

            // finding nearest hub en route too expensive for now - just use nearest hub
            pair<hubLocation, double> hDist = GlobalFlags::ch->findNearestHub(evPos);
            //# hub, hubDistance = GG.ch.findNearestHubDriving(evID)

            double evRange = 10000.0;
            double urgency = 0.0;
            if (ControlCentre::wUrgency > 0.0) {   // if we have an ugency weight then we need to calculate the range
                double distance = Vehicle::getDistance(evID);
                if (distance > 10000) {   // can compute real range after we've been driving for a while - arbitrary 10km
                    double mWh = distance / stod(Vehicle::getParameter(evID, "device.battery.totalEnergyConsumed"));
                    evRange = stod(Vehicle::getParameter(evID, "device.battery.actualBatteryCapacity")) * mWh / 1000.;
                    if (evRange <= 0.0)   // vehicle battery flat!
                        evRange = 1;
                }
                else   // otherwise just a guesstimate
                    evRange = stod(Vehicle::getParameter(evID, "device.battery.actualBatteryCapacity")) * ev.first->getMyKmPerWh();

                if (evRange <= 0.0)   // zero means battery flat!
                    evRange = 1.0;    // avoid divide by zero

                urgency = hDist.second/ evRange; //  we want most urgent to have lowest value - to be compatible with proximity (lowest proximity = nearest.

                if (urgency > maxUrgency)
                    maxUrgency = urgency;
            }

            double proximity = 0.0;
            if (ControlCentre::wEnergy > 0.0) {       // We have a weight so need to calculate proximity
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

                if (proximity > maxProximity)
                    maxProximity = proximity;
            }
            if (ControlCentre::wUrgency > 0.0)   // have urgency wt so want to determine urgency order
                EvAllocationFactorsList.insert(new EvAllocationFactors(urgency, urgency, proximity, ev.first, ev.second, 0));
            else   // zero urgency wt so just order by proximity
                EvAllocationFactorsList.insert(new EvAllocationFactors(proximity, urgency, proximity, ev.first, ev.second, 0));
        }

        // if urgency weight set we have a list sorted by urgency, if only proximity is set its sorted by proximity

        size_t evCount = EvAllocationFactorsList.size();
        double urgencyWt = 0.0;
        if (ControlCentre::wUrgency > 0.0) {                  // if urgency set - remember the position re urgency
            urgencyWt = ControlCentre::wUrgency / maxUrgency;
            if (evCount > 1) {
                int urgencyPosition = 0;
                for (auto urg : EvAllocationFactorsList) {
                    urg->setPosition(urgencyPosition);
                    urgencyPosition += 1;
                }
            }

            // if both weights set we need to calculate CEC and return a list sorted by CEC
            double proximityWt = 0.0;
            if (ControlCentre::wEnergy > 0.0) {
                proximityWt = ControlCentre::wEnergy / maxProximity;

                std::set <EvAllocationFactors*, decltype(EvAllocationFactorsCmp)* > CECList(EvAllocationFactorsCmp);
                for (const auto ef : EvAllocationFactorsList) {
                    ef->setCEC(urgencyWt, proximityWt);
                    CECList.insert(ef);
                }
                //EvAllocationFactorsListOut("CECList", CECList);
                return CECList;
            }
        }
    }  // otherwise return list,  will just be sorted by urgency or proximity depending on weights set
    return EvAllocationFactorsList;
}

/* find all the ev's that are requesting a charge and compute the mean distance to these
        note calling math.dist which will use sqrt is actually faster than comparing distances to the square
        we only update the ev positions on first call because we will repeat call to this fn for each ev in creating EvAllocationFactors list */
std::pair<std::unordered_set<EV*>, double> EvAllocationFactors::getNeighboursNeedingCharge(EV* ev, bool firstCall) {
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

std::ostream& EvAllocationFactors::operator<<(std::ostream& os) {
    os << CEC << "\t" << urgency << "\t" << proximity << "\t" << ev->getID() << "\t" << requestedCharge << "\t" << urgencyPosition; return os;
}


