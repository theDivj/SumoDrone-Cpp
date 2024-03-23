#pragma once
#include <iostream>
enum class EVState {
    DRIVING,
    CHARGEREQUESTED,
    WAITINGFORRENDEZVOUS,
    WAITINGFORDRONE,
    CHARGINGFROMDRONE,
    CHARGEBROKENOFF,
    LEFTSIMULATION,
    NULLSTATE
};
std::ostream& operator<<(std::ostream& os, EVState const& es);
