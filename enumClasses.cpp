#include <iostream>
#include "enumClasses.h"
#include <unordered_map>
#include "EV.h"

std::ostream& operator<<(std::ostream& os, enum DroneState const& ds) {
    switch (ds) {
    case DroneState::PARKED:
        os << "drPARKED"; return os;
    case DroneState::FLYINGTORENDEZVOUS:
        os << "drFLYINGTORENDEZVOUS"; return os;
    case DroneState::FLYINGTOEV:
        os << "drFLYINGTOEV"; return os;
    case DroneState::CHARGINGEV:
        os << "drCHARGINGEV"; return os;
    case DroneState::FLYINGTOCHARGE:
        os << "drFLYINGTOCHARGE"; return os;
    case DroneState::CHARGINGDRONE:
        os << "drCHARGINGDRONE"; return os;
    case DroneState::FLYINGTOPARK:
        os << "drFLYINGTOPARK"; return os;
    case DroneState::NULLSTATE:
        os << "drNULLSTATE"; return os;
    }
}

std::ostream& operator<<(std::ostream& os, enum EVState const& es) {
switch (es) {
    case EVState::DRIVING:
        os << "DRIVING"; return os;
    case EVState::CHARGEREQUESTED:
        os << "evCHARGEREQUESTED"; return os;
    case EVState::WAITINGFORRENDEZVOUS:
        os << "evWAITINGFORRENDEZVOUS"; return os;
    case EVState::WAITINGFORDRONE:
        os << "evWAITINGFORDRONE"; return os;
    case EVState::CHARGINGFROMDRONE:
        os << "evCHARGINGFROMDRONE"; return os;
    case EVState::CHARGEBROKENOFF:
        os << "evCHARGEBROKENOFF"; return os;
    case EVState::LEFTSIMULATION:
        os << "evLEFTSIMULATION"; return os;
    case EVState::NULLSTATE:
        os << "evNULLSTATE"; return os;
    }
}


