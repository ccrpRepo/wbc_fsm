#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include "common/mathTypes.h"
#include "common/mathTools.h"
#include "interface/CmdPanel.h"
#include "common/enumClass.h"

struct MotorState
{
	unsigned int mode;
    float q;
    float dq;
    float ddq;
    float tauEst;

    MotorState(){
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

struct IMU
{
    float quaternion[4];    // w, x, y, z
    float gyroscope[3];
    float accelerometer[3];

    IMU(){
        for(int i = 0; i < 3; i++){
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
    }

    RotMat getRotMat(){
        Quat quat;
        quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return quatToRotMat(quat);
    }

    Vec3 getAcc(){
        Vec3 acc;
        acc << accelerometer[0], accelerometer[1], accelerometer[2];
        return acc;
    }

    Vec3 getGyro(){
        Vec3 gyro;
        gyro << gyroscope[0], gyroscope[1], gyroscope[2];
        return gyro;
    }

    Quat getQuat(){
        Quat q;
        q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return q;
    }
};

struct LowlevelState
{
    IMU imu;
    MotorState motorState[29];
    UserCommand userCmd;
    UserValue userValue;


    RotMat getRotMat(){
        return imu.getRotMat();
    }

    Vec3 getAcc(){
        return imu.getAcc();
    }

    Vec3 getGyro(){
        return imu.getGyro();
    }

    Vec3 getAccGlobal(){
        return getRotMat() * getAcc();
    }

    Vec3 getGyroGlobal(){
        return getRotMat() * getGyro();
    }

    double getYaw(){
        return rotMatToRPY(getRotMat())(2);
    }

    double getDYaw(){
        return getGyroGlobal()(2);
    }

};

#endif  //LOWLEVELSTATE_HPP