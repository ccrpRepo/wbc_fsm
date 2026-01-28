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
    R2, // motion pause
    R1, // motion continue
    R2_A, // loco_mode
    R1_UP, //dance
    L2_B, // passive
    
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
};

#endif  // ENUMCLASS_H