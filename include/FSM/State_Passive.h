
#ifndef PASSIVE_H
#define PASSIVE_H

#include <nlohmann/json.hpp>
#include "FSMState.h"

#define NUM_DOF 29
using json = nlohmann::json;

class State_Passive : public FSMState{
public:
    State_Passive(CtrlComponents *ctrlComp);
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

    double _Kds = 10;
};

#endif  // PASSIVE_H