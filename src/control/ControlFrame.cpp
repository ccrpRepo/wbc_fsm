#include "control/ControlFrame.h"

ControlFrame::ControlFrame(CtrlComponents *ctrlComp):_ctrlComp(ctrlComp){
    _FSMController = new FSM(_ctrlComp);
} 

void ControlFrame::run(){

    _FSMController->run();
}