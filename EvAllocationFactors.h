#pragma once
#include <iostream>
#include <unordered_set>
#include <set>

#include <string>
#include "EV.h"
#include "ControlCentre.h"

class EvAllocationFactors;
bool EvAllocationFactorsCmp(const EvAllocationFactors* lurg, const EvAllocationFactors* rurg);

class EvAllocationFactors {
    friend class ControlCentre;

public:
    double CEC = 0.0;
    int urgencyPosition = 0;

protected:
    EV* ev;
    double requestedCharge;
    double urgency;
    double proximity;

    EvAllocationFactors(double c, double u, double p, EV* e, double r, int up) : CEC(c), urgency(u), proximity(p), ev(e), requestedCharge(r), urgencyPosition(up) {}
    ~EvAllocationFactors() {}

    bool operator<(const EvAllocationFactors& z) { return z.CEC < CEC; }
    bool operator==(const EvAllocationFactors& r) { return ev->getID() < r.ev->getID(); }
    std::ostream& operator<<(std::ostream& os);

public:
    static std::set<EvAllocationFactors*, decltype(EvAllocationFactorsCmp)* > calcEvAllocationFactors();
    static std::pair<std::unordered_set<EV*>, double> getNeighboursNeedingCharge(EV* ev, bool firstCall);
    std::string getID() { return ev->getID(); }
    void setPosition(int up) { urgencyPosition = up; }
    int getPosition() { return urgencyPosition; }
    void setCEC(double urgencyWt, double proximityWt) { CEC = urgencyWt * urgency + proximityWt * proximity;  }
};

