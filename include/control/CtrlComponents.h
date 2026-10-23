#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/CmdPanel.h"
#include <string>
#include <iostream>


struct CtrlComponents{
public:
    CtrlComponents(IOInterface *ioInter):ioInter(ioInter){
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState();
        exitFlag = false;
    }
    ~CtrlComponents(){
        delete lowCmd;
        delete lowState;
        delete ioInter;
    }
    LowlevelCmd *lowCmd;
    LowlevelState *lowState;
    IOInterface *ioInter;

    double dt;
    bool *running;
    bool exitFlag;
    CtrlPlatform ctrlPlatform;

    void recvState(){
        ioInter->recvState(lowState);
    }

    void sendCmd(){
        ioInter->sendCmd(lowCmd);
    }



};

#endif  // CTRLCOMPONENTS_H