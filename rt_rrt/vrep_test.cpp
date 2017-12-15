////
////  vrep_test.cpp
////  rt_rrt
////
////  Created by 谢龙 on 2017/12/15.
////  Copyright © 2017年 long. All rights reserved.
////
//
//#include <stdio.h>
//#include <stdlib.h>
//#include <iostream>
//#include <eigen3/Eigen/Geometry>
//#include "Manipulator.hpp"
//
//#include "VREP.hpp"
//#include "VREP_arm.hpp"
//
//int main()
//{
//    VREP v;
//    v.connect();
//    v.simStop();
//    if(v.clientID == -1)
//            return 0;
//    int h = v.getHandle("obstacle_1");
//    Eigen::MatrixXd position;
//    position = v.getPosition(h);
//    position = position * 1.3;
//    
//    v.simStart();
//    v.setPosition(h, position);
//    v.simSleep(10000);
//    v.simStop();
//
////    VREP_arm arm("goal", v.clientID, v.mode);
////
////    clock_t start = clock();
////    arm.getPosition();
////    clock_t ends = clock();
////    std::cout <<"Running Time : "<<(double)(ends - start)/ CLOCKS_PER_SEC << std::endl;
////
////    std::cout <<"**********************************************************"<< std::endl;
////
////    v.simStart();
////    float joint[] = {M_PI/2, 0, 0,  0,  0,  0,  0};
////    clock_t startgetPosition = clock();
////    arm.setJointPos(joint);
////    clock_t endsgetPosition = clock();
////    std::cout <<"Running Time : "<<(double)(endsgetPosition - startgetPosition)/ CLOCKS_PER_SEC << std::endl;
////
////    std::cout <<"**********************************************************"<< std::endl;
////    v.simSleep(3000);
////    v.simStop();
////
////    Eigen::MatrixXd manipulator_dh(7,4);
////    manipulator_dh <<  0, 0.20386, 0, M_PI/2,
////                0, 0, 0, -M_PI/2,
////                0, 0.29126, 0, M_PI/2,
////                0, 0, 0, -M_PI/2,
////                0, 0.32363,  0, M_PI/2,
////                0, 0, 0, -M_PI/2,
////                0, 0.15512, 0, 0;
////    Manipulator manipulator(manipulator_dh);
////    Eigen::MatrixXd joint_angle(7,1);
////    joint_angle << 90, 0, 45, 0, 0, 0, 0;
////
////    clock_t start_setJointPos = clock();
////    manipulator.fkine(joint_angle);
////    clock_t ends_setJointPos = clock();
////    std::cout <<"fkine Running Time : "<<(double)(ends_setJointPos - start_setJointPos)/ CLOCKS_PER_SEC << std::endl;
////
////    std::cout <<"**********************************************************"<< std::endl;
////
////    Eigen::MatrixXd goal_position(3, 1);
////    goal_position << -0.1498, -0.4, -0.2697;
////    clock_t start_ikine = clock();
////    manipulator.ikine(goal_position);
////    clock_t ends_ikine = clock();
////    std::cout <<"ikine Running Time : "<<(double)(ends_ikine - start_ikine)/ CLOCKS_PER_SEC << std::endl;
////
////    std::cout <<"**********************************************************"<< std::endl;
////
////    clock_t start_jacob = clock();
////    manipulator.jacob(joint_angle);
////    clock_t ends_jacob = clock();
////    std::cout <<"jacob Running Time : "<<(double)(ends_jacob - start_jacob)/ CLOCKS_PER_SEC << std::endl;
////
////    std::cout <<"**********************************************************"<< std::endl;
//    return 0;
//}
//
//
//
//
//
//
//
//
//
//
//
//
