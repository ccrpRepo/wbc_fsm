#ifndef IOINTERFACE_H
#define IOINTERFACE_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/CmdPanel.h"
#include <string>

class IOInterface{
public:
IOInterface(){}
~IOInterface(){delete cmdPanel;}
virtual void recvState(LowlevelState *state) = 0;
virtual void sendCmd(const LowlevelCmd *cmd) = 0;
void zeroCmdPanel(){cmdPanel->setZero();}
void setPassive(){cmdPanel->setPassive();}

protected:
CmdPanel *cmdPanel;
};

#endif  //IOINTERFACE_H