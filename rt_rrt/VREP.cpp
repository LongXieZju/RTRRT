//
//  VREP.cpp
//  rt_rrt
//
//  Created by 谢龙 on 2017/12/14.
//  Copyright © 2017年 long. All rights reserved.
//

#include "VREP.hpp"
#include <iostream>

extern "C" {
#include "extApi.h"
#include "extApi.c"
#include "extApiPlatform.c"
}

void VREP::connect(){
    //    simxChar* Adresse = "127.0.0.1";
    int Port = 19997;
    VREP::mode = simx_opmode_oneshot_wait;
    VREP::clientID = simxStart("127.0.0.1", Port, true,true,2000,5);
    if(VREP::clientID != -1){
        std::cout << "V-rep connected" << std::endl;
    }else{
        std::cout << "Can not connect V-rep" << std::endl;
        throw("Can not connect V-rep");
    }
}

void VREP::simStart(){
    simxStartSimulation(VREP::clientID, VREP::mode);
}

void VREP::simStop(){
    simxStopSimulation(VREP::clientID, VREP::mode);
}

void VREP::simPause(){
    simxPauseSimulation(VREP::clientID, VREP::mode);
}

void VREP::simSleep(int time){
    extApi_sleepMs(time);
}

int VREP::getHandle(const char* name){
    int handle;
    int s = simxGetObjectHandle(VREP::clientID, name, &handle, VREP::mode);
    if(s != 0){
        std::cout << "Can not get handle: "<< name << std::endl;
        throw("Can not get handle: %s", name);
    }
    return handle;
}

Eigen::MatrixXd VREP::getPosition(int handle){
    float position[3];
    int s = simxGetObjectPosition(VREP::clientID, handle, -1, position, VREP::mode);
    if(s != 0){
        std::cout << "Can not get Position" << std::endl;
        throw("Can not get Position");
    }
    Eigen::MatrixXd result(3,1);
    result(0,0) = position[0];
    result(1,0) = position[1];
    result(2,0) = position[2];
    return result;
}

void VREP::setPosition(int handle, Eigen::MatrixXd& position){
    float position_temp[3];
    position_temp[0] = position(0,0);
    position_temp[1] = position(1,0);
    position_temp[2] = position(2,0);
    int s = simxSetObjectPosition(VREP::clientID, handle, -1, position_temp, VREP::mode);
    if(s != 0){
        std::cout << "Can not Set Position" << std::endl;
        throw("Can not Set Position");
    }
}

void VREP::setJointPos(int handle, float joint){
    int s = simxSetJointTargetPosition(VREP::clientID, handle, joint, VREP::mode);
    if(s != 0)
        throw("Can not set joint position target");
}
