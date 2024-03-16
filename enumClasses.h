#pragma once
#include <iostream>
#include <set>




//class EV;
using namespace std;

struct urgency {
    pair<double, pair<EV* const, double> > urg;
};
/*
struct urgencyCompare {
    bool operator()(const pair<double, pair<EV* const, double> >& lUrg, const pair<double, pair<EV* const, double> >& rUrg) const {
        return true;
    }
}; */


std::ostream& operator<<(ostream& os, set<urgency>   const& urgency)
{
    for (auto& urgent : urgency)
        os << urgent.urg.first << "\t" << urgent.urg.second.first->getID() << "\t" << urgent.urg.second.second << "\n";
    return os;
}