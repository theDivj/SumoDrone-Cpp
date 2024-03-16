#pragma once
#include <unordered_map>
#include <libsumo/libsumo.h>

struct hubLocation {
    std::string id;
    libsumo::TraCIPosition xypos;
    std::string edge;
    double epos = 0;

    hubLocation(std::string pID, libsumo::TraCIPosition pPos, std::string pEdge, double pEpos) { id = pID; xypos = pPos; edge = pEdge; epos = pEpos; }

    hubLocation(double pos = 0.0) : epos{ pos } { }
    
};

class ChargeHubs
{
public:
    std::unordered_map<std::string, hubLocation> chargeHubLocations;

    ChargeHubs() { locateChargeHubs(); }

    std::pair<hubLocation, double> findNearestHub(libsumo::TraCIPosition pxy);

    std::pair<hubLocation, double> findNearestHubDriving(std::string evID);

    void locateChargeHubs();

    std::pair<hubLocation, double> nearestHubLocation(const libsumo::TraCIPosition pos);

 
};

