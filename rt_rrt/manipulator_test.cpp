//
//  manipulator_test.cpp
//  rt_rrt
//
//  Created by 谢龙 on 2017/12/14.
//  Copyright © 2017年 long. All rights reserved.
//

#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Geometry>

#include "Manipulator.hpp"

//int main(){
//    Eigen::MatrixXd manipulator_dh(7,4);
//    manipulator_dh <<  0, 0.20386, 0, M_PI/2,
//                0, 0, 0, -M_PI/2,
//                0, 0.29126, 0, M_PI/2,
//                0, 0, 0, -M_PI/2,
//                0, 0.32363,  0, M_PI/2,
//                0, 0, 0, -M_PI/2,
//                0, 0.15512, 0, 0;
//    Manipulator manipulator(manipulator_dh);
//
//    Eigen::MatrixXd joint_angle(7,1);
//    joint_angle << 90, 0, 45, 0, 0, 0, 0;
////    std::cout << ma.jacob(joint_angle) <<std::endl;
//    Eigen::MatrixXd goal_position(1, 3);
//    goal_position << -0.1498, -0.4, -0.2697;
//    std::cout << manipulator.ikine(goal_position) <<std::endl;
//
//    clock_t start_jacob = clock();
//    manipulator.jacob(joint_angle);
//    clock_t ends_jacob = clock();
//    std::cout <<"jacob Running Time : "<<(double)(ends_jacob - start_jacob)/ CLOCKS_PER_SEC << std::endl;
//
//    std::cout <<"**********************************************************"<< std::endl;
//
//    clock_t start_sampleNode = clock();
//    manipulator.sampleNode();
//    clock_t ends_sampleNode = clock();
//    srand((unsigned)time(0));
//    std::cout <<"sampleNode Running Time : "<<(double)(ends_sampleNode - start_sampleNode)/ CLOCKS_PER_SEC << std::endl;
//    std::cout << manipulator.sampleNode() << std::endl;
//    std::cout <<"*******"<< std::endl;
//    std::cout << manipulator.sampleNode() << std::endl;
//    std::cout <<"*******"<< std::endl;
//    std::cout << manipulator.sampleNode() << std::endl;
//
//    std::cout <<"**********************************************************"<< std::endl;
//
//}

