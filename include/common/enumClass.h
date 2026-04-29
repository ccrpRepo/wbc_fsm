#ifndef ENUMCLASS_H
#define ENUMCLASS_H

#include <iostream>
#include <sstream>

enum class CtrlPlatform{
    MUJOCO,
    REALROBOT,
};

enum class RobotType{
    A1,
    Go1
};

enum class UserCommand
{   
    NONE,
    SELECT, // exit,
    START, // fixed pose
    // F1,  // passive
    R2, // motion pause in setted refer idx
    R1, // motion continue
    R2_A, // loco_mode
    R1_UP, //wbc
    R1_LEFT, // wbcleft
    R1_RIGHT, // wbcright
    L2_B, // passive
    L2, // pause in current refer idx
    R2_UP, //high speed mode
    R2_DOWN, //low speed mode
    R2_B, // back to loco from amp
    
};

enum class FrameType{
    BODY,
    HIP,
    GLOBAL
};

enum class WaveStatus{
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};

enum class FSMMode{
    NORMAL,
    CHANGE
};

enum class FSMStateName{
    // EXIT,
    INVALID,
    PASSIVE,
    FIXEDSTAND,
    FIXEDDOWN,
    LOCO,
    WBC,
    WBCleft,
    WBCright,
    AMP,
    MJAMP,
};

#endif  // ENUMCLASS_H