#pragma once
enum class DroneState {  // Enumeration for Drone state mode
    PARKED,
    FLYINGTORENDEZVOUS,
    FLYINGTOEV,
    CHARGINGEV,
    FLYINGTOCHARGE,
    CHARGINGDRONE,
    FLYINGTOPARK,
    NULLSTATE
};
