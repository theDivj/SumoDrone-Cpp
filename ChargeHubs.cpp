#include "ChargeHubs.h"
#include <iostream>

using namespace libsumo;
using namespace std;

// find the nearest hub, as the crow flies, to a point (ev or drone location)
pair <hubLocation, double> ChargeHubs::findNearestHub(TraCIPosition pxy) {
    hubLocation minHub;
    double minDistance = numeric_limits<double>::max();
    for (auto& hub : chargeHubLocations) {
        double distance = pow(hub.second.xypos.x - pxy.x, 2) + pow(hub.second.xypos.y - pxy.y, 2);
        if (distance < minDistance) {
            minDistance = distance;
            minHub = hub.second;
        }
    }
    return { minHub, minDistance };
}

// find the nearest hub reachable by the ev, (driving distance)
pair <hubLocation, double> ChargeHubs::findNearestHubDriving(string evID) {
    hubLocation minHub;
    double minDistance = std::numeric_limits<double>::max();
    for (auto& hub : chargeHubLocations) {
        double dd = Vehicle::getDrivingDistance(evID, hub.second.edge, hub.second.epos);
        if (dd >= 0.0 && dd < minDistance) {
            minDistance = dd;
            minHub = hub.second;
        }
    }
    return { minHub, minDistance };
}

void ChargeHubs::locateChargeHubs() {
    auto chargeHubs = ChargingStation::getIDList();
    for (const auto& hub : chargeHubs) {
        string lane = ChargingStation::getLaneID(hub);
        string edge = lane.substr(0, lane.find("_"));
        double pos = ChargingStation::getStartPos(hub) + 5;
        TraCIPosition pxy = Simulation::convert2D(edge, pos);
        chargeHubLocations[hub] = hubLocation(hub, pxy, edge, pos);
        Route::add(edge, { edge });
    }
}

pair <hubLocation, double> ChargeHubs::nearestHubLocation(const TraCIPosition pos) {
    auto hub = findNearestHub(pos);
    return { chargeHubLocations[hub.first.id], hub.second };
}

