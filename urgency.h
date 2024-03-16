#pragma once
#include <iostream>
#include <unordered_set>
#include <set>

#include "EV.h"

#include "ControlCentre.h"
bool urgencyCmp(const urgency& lurg, const urgency& rurg);

class urgency {
public:
    double urg = 0.0;
    EV* ev;
    double requestedCharge;

    urgency(double u, EV* e, double r) : urg(u), ev(e), requestedCharge(r) {}
    bool operator<(const urgency& z) { return z.urg < urg; }
    bool operator==(const urgency& r) {
        return ev->getID() < r.ev->getID();
    }
    
    static std::set<urgency, decltype(urgencyCmp)* > calcUrgency();
    static std::pair<std::unordered_set<EV*>, double> getNeighboursNeedingCharge(EV* ev, bool firstCall);

};

std::ostream& operator<<(std::ostream& os, urgency const& urgent);
  