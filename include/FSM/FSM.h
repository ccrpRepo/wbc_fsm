#ifndef FSM_H
#define FSM_H

#include "FSM/FSMState.h"
#include "FSM/State_FixedStand.h"
#include "FSM/State_Passive.h"
#include "FSM/State_Loco.h"
#include "FSM/State_Amp.h"
#include "FSM/State_MJAmp.h"
#include "FSM/State_WBC.h"
#include "common/enumClass.h"
#include "control/CtrlComponents.h"

struct FSMStateList{
    FSMState *invalid;
    State_Passive *passive;
    State_FixedStand *fixedStand;
    State_Loco *loco;
    State_WBC *wbc;
    State_AMP *amp;
    State_MJAMP *mjamp;
    void deletePtr(){
        delete invalid;
        delete passive;
        delete fixedStand;
        delete loco;
        delete wbc;
        delete amp; 
        delete mjamp;
    }
};

class FSM{
public:
    FSM(CtrlComponents *ctrlComp);
    ~FSM();
    void initialize();
    void run();
private:
    FSMState* getNextState(FSMStateName stateName);
    CtrlComponents *_ctrlComp;
    FSMState *_currentState;
    FSMState *_nextState;
    FSMStateName _nextStateName;
    FSMStateList _stateList;
    FSMMode _mode;
    long long _startTime;
    int count;
};


#endif  // FSM_H