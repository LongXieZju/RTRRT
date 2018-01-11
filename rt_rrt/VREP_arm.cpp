//
//  VREP_arm.cpp
//  rt_rrt
//
//  Created by 谢龙 on 2017/12/14.
//  Copyright © 2017年 long. All rights reserved.
//

#include "VREP_arm.hpp"
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Geometry>

VREP_arm::VREP_arm(const char* name, int clientID, int mode){
    VREP_arm::clientID = clientID;
    VREP_arm::mode = mode;
    VREP_arm::getHandle(name);
    int handle;
    VREP_arm::link_num = 7;
    VREP_arm::link_handle[0] = VREP_arm::getHandle("redundantRob_joint1");
    VREP_arm::link_handle[1] = VREP_arm::getHandle("redundantRob_joint2");
    VREP_arm::link_handle[2] = VREP_arm::getHandle("redundantRob_joint3");
    VREP_arm::link_handle[3] = VREP_arm::getHandle("redundantRob_joint4");
    VREP_arm::link_handle[4] = VREP_arm::getHandle("redundantRob_joint5");
    VREP_arm::link_handle[5] = VREP_arm::getHandle("redundantRob_joint6");
    VREP_arm::link_handle[6] = VREP_arm::getHandle("redundantRob_joint7");
}

Eigen::MatrixXd VREP_arm::getPosition(){
    Eigen::MatrixXd position(VREP_arm::link_num, 3);
    for(int i = 0; i < VREP_arm::link_num; i++){
        position.row(i) = VREP::getPosition(VREP_arm::link_handle[i]);
    }
    return position;
}

void VREP_arm::setJointPos(float* joint){
    for(int i = 0; i < VREP_arm::link_num; i++){
        VREP::setJointPos(VREP_arm::link_handle[i], joint[i]);
    }
}

void VREP_arm::setJointPos(Eigen::MatrixXd joint){
    for(int i = 0; i < VREP_arm::link_num; i++){
        VREP::setJointPos(VREP_arm::link_handle[i], joint(i, 0));
    }
}

Eigen::MatrixXd VREP_arm::getJointAngle(){
    Eigen::MatrixXd position(VREP_arm::link_num, 1);
    for(int i = 0; i < VREP_arm::link_num; i++){
        float q = VREP::getJointPos(VREP_arm::link_handle[i]);
        position.row(i) << q;
    }
    return position;
}
