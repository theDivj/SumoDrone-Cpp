Simple example to model the use of Drones to charge Electric vehicles (EVs).

  Basically vehicles drive until they need a charge - currently triggered by thresholds in the EV model
  they then get charged by the nearest drone, that is not busy, after which the drone will return to the nearest charge hub - or another vehicle if required
  
Charging stations(need to be created on the road network and are 'discovered' in the code. These are used to launch and 'charge' drones
The SUMO default for these is that the charging power is 0 - so they won't charge vehicles even if the vehicles stop at the station.

This code uses the nearest free drone, or if none free, then one will be spawned (upto a limit) from the nearest charging station.
When charging is complete/the drone is out of charge then the drone goes to the nearest chargeing station to recharge.

The parameters used for the Drone in this model correspond to those of an Ehang 184 which has top speed of 60km/h, and a 14.4 KW battery giving 23 mins flight time.
The order of allocation of drones to vehicles is dependant on the 'urgency' the ratio between the distance to the nearest charge point and the remaining charge

In the GUI, vehicles and drones turn red when they need charging and green when they are actually charging.


Execution:
   open a cmd/shell window in the directory where these files have been placed
     python drclass.py -h      summarises the runstring parameters and defaults
     
   demonstration files : (running from the directory where these files were unpacked.)
      python  drclass.py  demo/demo.sumocfg                   Note that the first drone is launched around 1200s
     
      
Files:
    drclass.py          Startup file - parameter processing
    ControlCentre.py    Control Centre class - handling requests for charge and allocation of drones
    Simulation.py       Simulation class - mapping the insertion and departure of vehicles in the SUMO model
    Drone.py            Drone class - implementing the Drone state model, using a SUMO POI to represent a drone
    EV.py               EV class - implementing the EV state model, EVs in this class 'shadow' EVs in the SUMO model
    ChargeHubs.py       ChargeHubs class - static class maintaining charging station locations with location helper functions
    GlobalClasses.py    GlobalClasses - supporting communication between Control Centre, Drones and EVs
    DroneType.py        Drone Type class - holding the variables that define a drone behaviour - initialised in add.xml files
    drone.png           "Drone" image file 
    
    Demo                Directory containing a SUMO model with grid and traffic generated by randomTrips.py.
    Docs                Directory containing pDoc generated class documentation
    
    
Drone State model:
        PARKED                  Drone is parked at a charging hub, topping up batteries and available for allocation
        FLYINGTORENDEZVOUS*     Drone is flying to a computed rendezvous point for a specific EV
        FLYINGTOEV*             Drone is flying to an EV - either directly or after reaching the rendezvous point
        CHARGINGEV*             Drone is coupled to the EV and charging the EV
        FLYINGTOPARK            Drone is flying back to nearest Hub - is available for allocation
        FLYINGTOCHARGE          Drone is flying back to the hub, needing a charge of either the flying or EV charging battery
        CHARGINGDRONE           Drone is at a charging hub, charging to levels viable for allocation - cannot be allocated in this state
        NULLState               Drone is parked at a charging hub with full charges
            * states are managed by the EV, other states managed by the ControlCentre
                State changes can be triggered by the Drone battery levels running too low and the allocated EV leaving the simulation
                rendezvous states may be skipped using the -l (line of sight) parameter
        
EV State model:
        DRIVING                 EV is driving round the network - does not require a charge
        CHARGEREQUESTED         EV has requested a charge via the control centre
        WAITINGFORRENDEZVOUS    EV has been allocated a drone by the Control centre and is travelling to the computed rendezvous point
        WAITINGFORDRONE         Drone has reached the rendezvous point, EV is still driving, Drone is chasing to couple for charging     
        CHARGINGFROMDRONE       EV is charging from the Drone
        CHARGEBROKENOFF         Charge was broken off either because Drone broke off the charge or the vehicle left the simulation
        LEFTSIMULATION          EV has left the simulation
        NULL                    EV has left the simulation and told any Drone that it has left
                State changes are triggered by monitoring the EVs via traci,
                   by the Control Centre allocating a drone and by the drone breaking off to charge itself
        

At the end of the Simulation,  summary statistics are provided - as below, these are condensed into a single line if the -b (brief) option is used.


Summary Statistics:              2024-02-25T19:49:14.300185
        Model flags:    Rendezvous: True        Charge Once: False      Drone print: False
        Energy Weight: 1.0      Urgency Weight: 0.0     Proximity radius(m): 1000       Time steps: 15148.0

        Drone Totals:   (3 drones)
                Distance Km:    7.03                    The total distance flown by all drones.
                Flying KWh:     4.40                    The total KWh used to fly the drone
                Charging KWh:   57.06                   The total KWh used by the drone to charge EVs
        Drone Charger usage:
                Flying KWh:     4.44                    The charge provided by hubs to the Drone flying battery
                Charge KWh:     54.15                   The charge provided by hubs to the Drone EV charging battery
        Residuals:
                Flying KWh:     43.2                    The charge level of the flying battery at the end of the simulation
                Charging KWh:   87.1                    The charge level of the EV charging battery at the end of the simulation

        EV Totals:      (6 EVs)
                Charge KWh:     57.1                    The total charge provided to all EV's
                Charge Gap KWh: 0.5                     The sum of the difference between charge level and 'charge done' level for all EVs at simulation end
                Charge Sessions:
                        Full charges:   25              The number of successful charge sessions
                        Part (drone):   0               The number of sessions broken off because the Drone battery got too low
                        Part (ev):      1               The number of sessions broken off because the EV left the simulation while charging

        Successful chases: 26   Average chase time: 3.4s        broken Chases: 0              Chases represent the time between rendezvous and actually reaching the EV
                                                                                                   ( reflects the performance of the rendezvous computation.)
Discrete Drone data:
        drone:d1        Km:2.73 Charge KW:19.83 FlyingKW:1.71   Residual ( chargeWh:30021 flyingWh:14418 )
        drone:d2        Km:1.85 Charge KW:17.43 FlyingKW:1.16   Residual ( chargeWh:30007 flyingWh:14408 )
        drone:d3        Km:2.45 Charge KW:19.80 FlyingKW:1.53   Residual ( chargeWh:27056 flyingWh:14408 )


/demo/demo.add.xml  contains examples of drone definitions - these are all  POI definitions - using type "drone" to identify them.
                    A d0 definition is used to provide defaults for drones generated by SumoDrone and for other drones in the app.xml file.
                    other "drone" entries in the file are only used when the --z  (zeroDrone) option is used.




