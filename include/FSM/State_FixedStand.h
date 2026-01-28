
#ifndef FIXEDSTAND_H
#define FIXEDSTAND_H

#include <nlohmann/json.hpp>
#include "FSM/FSMState.h"

#define NUM_DOF 29
using json = nlohmann::json;

class State_FixedStand : public FSMState{
public:
    State_FixedStand(CtrlComponents *ctrlComp);
    ~State_FixedStand(){}
    void enter();
    void run();
    void exit();
    FSMStateName checkChange();

private:

    float _targetPos[NUM_DOF] = {-0.2, 0.0, 0.0, 0.42, -0.23, 0.0,
                              -0.2, 0.0, 0.0, 0.42, -0.23, 0.0,
                            0.0, 0.0, 0.0, 
                        0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0,
                        0.35, -0.18, 0.0, 0.87, 0.0, 0.0, 0.0,}; 

    float _startPos[NUM_DOF];
    float _duration = 2.0;
    float _phase = 0;  
    bool _fixedstand_complete_flag;

    float Kps[NUM_DOF] = {100, 100, 100, 150, 40, 40,
                        100, 100, 100, 150, 40, 40,
                        300, 300, 300,
                        100, 100, 50, 50, 20, 20, 20,
                        100, 100, 50, 50, 20, 20, 20,};
    float Kds[NUM_DOF] = {2, 2, 2, 4, 2, 2,
                        2, 2, 2, 4, 2, 2,
                        3, 3, 3,
                        2, 2, 2, 2, 1, 1, 1,
                        2, 2, 2, 2, 1, 1, 1,};
};

#endif  // FIXEDSTAND_H