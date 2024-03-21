#pragma once
#include <iostream>
#include <fstream>
#include <cstdlib>
#include "ControlCentre.h"
#include "ChargeHubs.h"
#include "dSimulation.h"

class dSimulation;
class ChargeHubs;
class ControlCentre;

class GlobalFlags
{
public:
    static inline dSimulation* ss;
    static inline ChargeHubs* ch;
    static inline ControlCentre* cc;

public:
    static inline bool myModelRendezvous;   // = True whether we estimate a rendezvous point for drone / ev
    static inline bool myOnlyChargeOnce;    // = True   # whether we are allowed to charge EVs more than once in a simulation
    static inline bool myChargePrint;       // = False     # Whether to print a charging log

    static inline std::ofstream myChargeLog;      // = None        # Filename for the charging log
    static inline bool myDronePrint;              // = False      # Whether to print a drone activity log
    static inline std::ofstream myDroneLog;       // = None         # Filename for the drone log

    static inline double myDroneKmPerHr;              // = 60.0     # default drone speed - to allow command line override
    static inline bool myUseRandom;                   // = False       # whether to generate 'random' charge requests

    GlobalFlags(dSimulation* pss, ChargeHubs* pch, ControlCentre* pcc) { ss = pss; ch = pch; cc = pcc; }
    GlobalFlags() = default;

    ~GlobalFlags();

    static const inline double getDroneSpeed(void) {       // getter for droneKmPerHr
        return myDroneKmPerHr;
    }

    double getRandom(void) {    // invoke my generator!
        return rand();
    }

    static void setGlobals(double droneKmPerHr, unsigned int randomSeed, std::string droneLog, std::string chargeLog, bool onlyChargeOnce, bool modelRendezvous);


    static  inline const bool usingRandom() {              // returns True if we are using random nos, False otherwise"""
        return GlobalFlags::myUseRandom;
    }
};

