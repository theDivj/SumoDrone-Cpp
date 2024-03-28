#pragma once
#include <unordered_map>
#include <libsumo/libsumo.h>

struct hubLocation {
    std::string id;
    libsumo::TraCIPosition xypos;
    std::string edge;
    double epos = 0;
    double ePower = 75000;  // effective power

    hubLocation(std::string pID, libsumo::TraCIPosition pPos, std::string pEdge, double pEpos, double pPower = 75000) {
        id = pID; xypos = pPos; edge = pEdge; epos = pEpos; ePower = pPower;
    }

    hubLocation(double pos = 0.0) : epos{ pos } { }   
};

class ChargeHubs
{
    std::unordered_map<std::string, hubLocation> chargeHubLocations;

public:
    ChargeHubs() { locateChargeHubs(); }

    std::pair<hubLocation, double> findNearestHub(libsumo::TraCIPosition pxy);

    std::pair<hubLocation, double> findNearestHubDriving(std::string evID);

    void locateChargeHubs();

    std::pair<hubLocation, double> nearestHubLocation(const libsumo::TraCIPosition pos);

};

