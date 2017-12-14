//
//  VREP_arm.hpp
//  rt_rrt
//
//  Created by 谢龙 on 2017/12/14.
//  Copyright © 2017年 long. All rights reserved.
//

#ifndef VREP_arm_hpp
#define VREP_arm_hpp

#include "VREP.hpp"

class VREP_arm: public VREP{
public:
    int link_num;
    int link_handle[7];
public:
    VREP_arm(const char* name, int clientID, int mode);
    Eigen::MatrixXd getPosition();
    void setJointPos(float *joint);
    void setJointPos(Eigen::MatrixXd joint);
};

#endif /* VREP_arm_hpp */
