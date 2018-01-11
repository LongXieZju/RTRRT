//
//  VREP.hpp
//  rt_rrt
//
//  Created by 谢龙 on 2017/12/14.
//  Copyright © 2017年 long. All rights reserved.
//

#ifndef VREP_hpp
#define VREP_hpp

#include <eigen3/Eigen/Geometry>

class VREP{
public:
    int clientID;
    int mode;
    
public:
    void connect();
    void simPause();
    void simStart();
    void simStop();
    void simSleep(int time);
    int getHandle(const char* name);
    Eigen::MatrixXd getPosition(int handle);
    void setPosition(int handle, Eigen::MatrixXd& position);
    void setJointPos(int handle, float joint);
    float getJointPos(int handle);
};

#endif /* VREP_hpp */
