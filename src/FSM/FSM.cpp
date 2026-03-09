#include "FSM/FSM.h"
#include <iostream>  


FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){
    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.loco = new State_Loco(_ctrlComp);
    _stateList.amp = new State_AMP(_ctrlComp);
    _stateList.wbc = new State_WBC(_ctrlComp);
    initialize(); 
}

FSM::~FSM(){  
    _stateList.deletePtr();
}

void FSM::initialize(){
    _currentState = _stateList.passive;
    _currentState -> enter();  
    _nextState = _currentState;
    _mode = FSMMode::NORMAL;  

    std::cout<<"Press **start** to enter position control mode..."<<std::endl;
}

void FSM::run(){
    try{
        _startTime = getSystemTime();  
        
        _ctrlComp->sendRecv(); 

        if(_mode == FSMMode::NORMAL){  
            _currentState->run();  
            _nextStateName = _currentState->checkChange();    
            if(_nextStateName != _currentState->_stateName){  
                _mode = FSMMode::CHANGE;  
                _nextState = getNextState(_nextStateName); 
                std::cout << "Switched from " << _currentState->_stateNameString
                << " to " << _nextState->_stateNameString << std::endl; 
            }
        }
        else if(_mode == FSMMode::CHANGE){  
            _currentState->exit();  
            _currentState = _nextState; 
            _currentState->enter();  
            _mode = FSMMode::NORMAL; 
            _currentState->run(); 
        }

        absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));  
    }catch (const std::exception& e) {
        std::cerr << std::endl << "Caught exception: " << e.what() << std::endl;
        _ctrlComp->exitFlag = true;
    }
}

FSMState* FSM::getNextState(FSMStateName stateName){  
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::LOCO: 
        return _stateList.loco;
    case FSMStateName::WBC:
        return _stateList.wbc;
    case FSMStateName::AMP:
        return _stateList.amp;
    default:
        return _stateList.invalid;
        break;
    }
}