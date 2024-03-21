// SumoDrone.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
#include "stdlib.h"

#include <libsumo/libsumo.h>

#include "SumoDrone.h"
#include "argparse.h"
#include "Drone.h"
#include "GlobalFlags.h"
#include "dSimulation.h"


using namespace libsumo;
using namespace std;

SumoDrone::SumoDrone() {
    briefStatistics = false;
    maxEVs = numeric_limits<int>::max();
    gg = nullptr;

    string sumoHome = std::getenv("SUMO_HOME");
    if (sumoHome.empty()) {    // append sumohome/tools to the path (removed for now)
        cout << "SUMO_HOME not set in environment, cannot execute sumo/sumo-gui" << endl;
        exit(1);
    }
}

SumoDrone::~SumoDrone() {
    gg->cc->printDroneStatistics(briefStatistics, this -> version, this->runstring);
}

GlobalFlags* SumoDrone::parseRunstring(int argc, char* argv[]) {

    argparse::ArgumentParser program(argv[0], "v3.1");
    // set up the expected runstring
    program.add_argument("sumocfg")               // mandatory
        .help("sumo configuration file");

    program.add_argument("-b", "--brief")
        .help("output a single line statistics summary, default full summary").flag();

    program.add_argument("-c", "--chargeFile")
        .help("file for output of detailed EV charge levels beginning/end of charge, default no output")
        .default_value("");

    program.add_argument("-d", "--maxDrones")
        .help("maximum drones to spawn, default is 6")
        .default_value(6).scan<'i', int>();

    program.add_argument("-e", "--maxEVs")
        .help("maximum EVs that are allowed to charge by Drone, default is no limit")
        .default_value(numeric_limits<int>::max()).scan<'i', int>();

    program.add_argument("-k", "--droneKmPerHr")
        .help("drone speed Km/h default = 60.0")
        .default_value(60.0).scan<'g', double>();

    program.add_argument("-l", "--lineOfSight")
        .help("route drone to EV by line of sight at each step, default is to compute a rendezvous point").flag();

    program.add_argument("-m", "--multipleCharge")
        .help("Allow EVs to be charged more than once - default is only once").flag();

    program.add_argument("-o", "--droneLog")
        .help("file for output of detailed drone charge levels for each step, default no output")
        .default_value("");

    program.add_argument("-p", "--proximityRadius")
        .help("proximity radius to scan for vehicles needing charge, default 1000")
        .default_value(1000.0).scan<'g', double>();

    program.add_argument("-r", "--randomSeed")
        .help("seed for random generator triggering requests and sizeof requests")
        .default_value(0).scan<'i', int>();

    program.add_argument("-s", "--sumoBinary")
        .help("sumo binary to execute against configuration, default is sumo")
        .default_value("sumo");

    program.add_argument("-t", "--droneType")
        .help("type of drone - currently ehang184 (default) or ehang184x")
        .default_value("ehang184");

    program.add_argument("--we", "--wEnergy")
        .help("weighting to apply to vehicles found in radius, default 1")
        .default_value(1.0).scan<'g', double>();

    program.add_argument("--wu", "--wUrgency")
        .help("weighting to apply to nearest vehicle urgency, default 0")
        .default_value(0.0).scan<'g', double>();
   
    program.add_argument("--z", "--zeroDrone")
        .help("Only use drones defined in the ...add.xml file").flag();

    try {
        program.parse_args(argc, argv);
    }
    catch (const std::exception& err) {
        std::cerr << err.what() << std::endl;
        std::cerr << program;
        exit(1);
    }

    bool modelRendezvous = true;
    if (program["lineOfSight"] == true)
        modelRendezvous = false;

    bool onlyChargeOnce = true;
    if (program["multipleCharge"] == true)
        onlyChargeOnce = false;

    bool zeroDrone = false;
    if (program["zeroDrone"] == true)
        zeroDrone = true;


    auto chargeLog = program.get<string>("--chargeFile");
    auto droneLog = program.get<string>("--droneLog");
    briefStatistics = program.get<bool>("--brief");
    int maxEVs = program.get<int>("--maxEVs");
    auto droneType = program.get<std::string>("--droneType");
    auto randomSeed = program.get<int>("--randomSeed");
    auto maxDrones = program.get<int>("--maxDrones");
    auto droneKmPerHr = program.get<double>("--droneKmPerHr");
    auto proximityRadius = program.get<double>("--proximityRadius");
    auto sumoBinary = program.get<std::string>("--sumoBinary");
    auto sumocfg = program.get<std::string>("sumocfg");
    auto wEnergy = program.get<double>("--wEnergy");
    auto wUrgency = program.get<double>("--wUrgency");

    //string sumoHome = getenv("SUMO_HOME");
    vector<string> sumoCmd = { sumoBinary, "-c", sumocfg };

    // create our management objects 
    dSimulation* ss = new dSimulation(sumoCmd, maxEVs);
    ChargeHubs* ch = new ChargeHubs();
    ControlCentre* cc = new ControlCentre(wEnergy, wUrgency, proximityRadius, maxDrones);

    // setup globals
    gg = new GlobalFlags(ss, ch, cc);
    gg->setGlobals(droneKmPerHr, randomSeed, droneLog, chargeLog, onlyChargeOnce, modelRendezvous);

    Drone::setDroneType(droneType);                         // set type as passed in runstring
    int poiDrones = Drone::setDroneTypeFromPOI(zeroDrone);  // set type(s) as set in POI definitions
    if (zeroDrone) {
        if (poiDrones > 0)
            cc->setMaxDrones(poiDrones);
        else
            cerr << "Warning: No drones found in POI - defaulting to " << maxDrones << " drones" << endl;
    }

    // any output file would have been opened in parse_args() - write out the title line if needed
    if (GlobalFlags::myDronePrint)
        GlobalFlags::myDroneLog << "timeStep\tDrone\tEV\tLane\tPosition\tdrone x\tdrone y\tdroneWh\tchargeWh\tflyingWh\tactivity" << endl;
    if (GlobalFlags::myChargePrint)
        GlobalFlags::myChargeLog << "timeStep\tEV id\tEV State\tDrone\tCapacity\tChargeWh(if_any)\tRequestedChargeWh" << endl;

    return gg;
}

void SumoDrone::loop() const {
    while (gg->ss->dStep())
        continue;     // do nothing
}

int main(int argc, char* argv[]) {
/*
    Simulation::start({ "sumo", "-c", "E:/SUMODrone/demo/demo.sumocfg" });
    for (int i = 0; i < 5000; i++) {
        Simulation::step();
        std::cout << "step" << std::endl;
    }
    Simulation::close();
  */  
    SumoDrone* session = new SumoDrone;
    string runstring;
    for (int i = 0; i < argc; i++)
       runstring += std::string(" ") + std::string(argv[i]);

    session->runstring = runstring;
    session->version = "v3.3 20th March 2024";

    GlobalFlags* gg = session->parseRunstring(argc, argv);

    session->loop();

    delete session;
    delete gg;

    return 0;
}
//using namespace libsumo;

/* int main(int argc, char* argv[]) {
    Simulation::start({ "sumo", "-c", "E:/SUMODrone/demo/demo.sumocfg" });
    for (int i = 0; i < 5; i++) {
        Simulation::step();
    }
    Simulation::close();
    */

    // Run program: Ctrl + F5 or Debug > Start Without Debugging menu
    // Debug program: F5 or Debug > Start Debugging menu

    // Tips for Getting Started: 
    //   1. Use the Solution Explorer window to add/manage files
    //   2. Use the Team Explorer window to connect to source control
    //   3. Use the Output window to see build output and other messages
    //   4. Use the Error List window to view errors
    //   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
    //   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
