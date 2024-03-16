#include "GlobalFlags.h"

using namespace libsumo;
using namespace std;

void GlobalFlags::setGlobals(double droneKmPerHr, unsigned int randomSeed, std::string droneLog, std::string chargeLog, bool onlyChargeOnce, bool modelRendezvous) {
    // initialise globals used across drone,ev,controlcentre"""
    myDroneKmPerHr = droneKmPerHr;
    myOnlyChargeOnce = onlyChargeOnce;
    myModelRendezvous = modelRendezvous;

    if (randomSeed != 0) {
        myRandom = std::default_random_engine(randomSeed);
        myUseRandom = true;
    }

    if (!droneLog.empty()) {
        myDroneLog.open(droneLog);
        if (myDroneLog.is_open()) {
            myDroneLog << std::fixed;
            myDronePrint = true;
        }
    }

    if (!chargeLog.empty()) {
        myChargeLog.open(chargeLog);
        if (myChargeLog.is_open()) {
            myChargeLog << std::fixed;
            myChargePrint = true;
        }
    }
}

GlobalFlags::~GlobalFlags() {
    delete cc;
    delete ch;
    delete ss;

    if (myDroneLog.is_open())
        myDroneLog.close();
    if (myChargeLog.is_open())
        myChargeLog.close();
}