#include "dSimulation.h"
#include "GlobalFlags.h"

using namespace libsumo;
using namespace std;

#include "EV.h"
#include "Drone.h"
#include <iostream>

dSimulation::dSimulation(const vector<string> sumoCmd, int pmaxEVs) {

    try {
        Simulation::start(sumoCmd);
    } catch (const std::exception& err) { cerr << "start failed: " << err.what() << endl; }
    stepSecs = Simulation::getDeltaT();
    maxEVs = pmaxEVs;
    if (Simulation::getOption("chargingstations-output").length() > 1)
        useChargeHubs = true;
    else
        useChargeHubs = false;

    usingSumogui = false;   // see if we are using sumo-gui/sumo-gui.exe
    for (string str : sumoCmd)
        if (str.find("sumo-gui") != string::npos)
            usingSumogui = true;
}

bool dSimulation::dStep() {        //Simulation step
    if (Simulation::getMinExpectedNumber() > GlobalFlags::cc->insertedDummies) {
        Simulation::executeMove();                     //  move vehicles first so we can move drones to the same position
        timeStep += 1;

        if (!usingSumogui) {    // let them know we're working
            int op = int(timeStep / 200) * 200;
            if (op == timeStep) { 
                cerr << ".";
                op = int(timeStep / 16000) * 16000;   // new line every 80 dots
                if (op == timeStep)
                    cerr << endl;
            }
        }

        if (Simulation::getLoadedNumber() > 0) {
            auto loadedVehicles = Simulation::getLoadedIDList();     // add new EVs to our management list upto the maximum allowed
            for (const auto& vehID : loadedVehicles) {
                if (Vehicle::getParameter(vehID, "has.battery.device") == "true") // we are only interested in EVs
                    if (EVs.size() < maxEVs) {
                        EVs[vehID] = new EV(vehID);   // can set kmPerWh here to cater for different EVs - get from an EV parameter ?
                    }
            }
        }
        /*
        auto tlist = Simulation::getStartingTeleportIDList();
        if (tlist.size() > 0) {
            for (const auto& tport : tlist) {
                if (tport.find("-CB") != string::npos or tport.find("-FB") != string::npos) {
                    cerr << "tported: " << tport << endl;
                    tports.insert(tport);
                    }
                }
        }*/

        if (Simulation::getArrivedNumber() > 0) {             // handle vehicles that have left the simulation
            vector <string> arrivedVehicles = Simulation::getArrivedIDList();
            for (const auto& aID : arrivedVehicles) {
                if (EVs.find(aID) != EVs.end()) {
                    EVs[aID]->leftSimulation();       // notify EV shadow that the vehicle has left
                    EVs[aID]->update();               //  run the update as we will be removing this from the management loop
                    EVs.erase(aID);
                }
            }
        }


        for (auto& vehID : EVs) { // run the update(state machine) for each EV  we are managing
            vehID.second->update();
        }
        GlobalFlags::cc->update();           // trigger control centre management on this step               
        Simulation::step();                        // complete the SUMO step
        
        return true;
    }

    return false;
}
