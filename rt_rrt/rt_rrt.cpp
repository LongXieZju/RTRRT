//
//  rt_rrt.cpp
//  rt_rrt
//
//  Created by 谢龙 on 2017/12/14.
//  Copyright © 2017年 long. All rights reserved.
//

#include <iostream>
#include <math.h>
#include <eigen3/Eigen/Geometry>
#include "Manipulator.hpp"
#include "VREP.hpp"
#include "VREP_arm.hpp"

Eigen::MatrixXd getObstaclesPos(int* obstacles, VREP v){
    int num = sizeof(obstacles)/sizeof(*obstacles) + 1;
    Eigen::MatrixXd pos(3, num);
    for(int i = 0; i < num; i++){
        pos.col(i) = v.getPosition(obstacles[i]);
    }
    return pos;
}

int main(){
    srand((unsigned)time(0));  //random sample differently every time

    ///test
//    Eigen::MatrixXf A;
//    Eigen::Matrix<float, 1, 7, Eigen::RowMajor> B;
//    B << 1,2,3,4,5,6,7;
//    A = B;
//    std::cout << A << std::endl;
//    Eigen::MatrixXd mat = Eigen::MatrixXd::Random(1,3);

//    std::cout <<  (A.array() > 0).matrix()(1,1) << std::endl;
    ///

    // Manipulator model, contains forward and backward kinematics models
    Eigen::MatrixXd manipulator_dh(7,4);
    manipulator_dh <<  0, 0.20386, 0, M_PI/2,
    0, 0, 0, -M_PI/2,
    0, 0.29126, 0, M_PI/2,
    0, 0, 0, -M_PI/2,
    0, 0.32363,  0, M_PI/2,
    0, 0, 0, -M_PI/2,
    0, 0.15512, 0, 0;
    Eigen::MatrixXd joint_start(7,1);
    joint_start << 0, 0, 0, M_PI/2, 0, M_PI/2, 0;
    Manipulator seven_arm(manipulator_dh);

    // VREP, to gain environment info
    VREP v;
    v.simStop();
    v.connect();
    Eigen::MatrixXd goal_position = v.getPosition(v.getHandle("goal"));
    std::cout << "****goal_position****" << std::endl;
    std::cout << goal_position.transpose() << std::endl;
    int obstacles[3] = {v.getHandle("obstacle_1"), v.getHandle("obstacle_2"), v.getHandle("obstacle_3")};
    std::cout << "****obs_handle****" << std::endl;
    std::cout << obstacles[0] << "," << obstacles[1] << "," << obstacles[2] << std::endl;
    Eigen::MatrixXd obs_position = getObstaclesPos(obstacles, v);
    
    // VREP manipulator model
    VREP_arm v_arm("redundantRob", v.clientID, v.mode);


    // Setting up
    seven_arm.setGoalPosition(goal_position);
    seven_arm.setStartState(joint_start);
    std::cout << "****goal_angle****" << std::endl;
    std::cout << seven_arm.goal_angle.transpose() << std::endl;
    std::cout << "****start_angle****" << std::endl;
    std::cout << seven_arm.start_angle.transpose() << std::endl;
    std::cout << "****obs_position****" << std::endl;
    std::cout << obs_position.transpose() << std::endl;
    
    //KD-tree
    std::vector<std::vector<int> > index;
    std::vector<std::vector<int> > neighbors;
    std::vector<std::vector<double> > dist;
    flann::Matrix<double> point(joint_start.data(), 1, seven_arm.link_num);
    flann::Index<flann::L2<double>> kd_tree(point, flann::KDTreeIndexParams(4));
    kd_tree.buildIndex();


    //RRT
    Eigen::MatrixXd new_node(seven_arm.link_num,1);
    Eigen::MatrixXd neighbor_nodes;
    Eigen::MatrixXd nodes = Eigen::MatrixXd::Zero(7, seven_arm.max_iter);
    NearestNode nearest_node;
    int new_node_ind;
    int nearest_node_ind;
    double min_dist;

    //v.simStart();
    int count = 0;
    clock_t start_jacob = clock();
    for(int i = 0; i < seven_arm.max_iter; i++){
        count++;
        new_node = seven_arm.sampleNode();
        flann::Matrix<double> query(new_node.data(), 1, 7);
        kd_tree.knnSearch(query, index, dist, 1, flann::SearchParams(32, 0, false));
        nearest_node_ind = index[0][0];
        min_dist = dist[0][0];
        if(min_dist  > seven_arm.node_max_step){
            seven_arm.steer(new_node, nearest_node_ind);
        }
        if(seven_arm.obstacleCollision(new_node, nearest_node_ind, obs_position)){
            flann::Matrix<double> query(new_node.data(), 1, 7);
            kd_tree.radiusSearch(query, neighbors, dist, seven_arm.node_max_step, flann::SearchParams(32, 0, false));
            seven_arm.chooseParent(new_node, neighbors, nearest_node_ind, obs_position, new_node_ind);
            seven_arm.rewire(new_node, neighbors, obs_position);
//            seven_arm.insertNode(new_node, nearest_node_ind, new_node_ind);
            flann::Matrix<double> node(seven_arm.tree.col(new_node_ind).data(), 1, 7);
            kd_tree.addPoints(node);
//            if((new_node - seven_arm.goal_angle).norm() < seven_arm.node_max_step){
//                if(seven_arm.obstacleCollision(new_node, seven_arm.goal_angle, obs_position)){
//                    std::cout << "Find path" << std::endl;
//                    seven_arm.findPath(new_node_ind);
//                    break;
//                }
//            }
        }
    }
    clock_t ends_jacob = clock();
    std::cout <<"RRT Running Time : "<<(double)(ends_jacob - start_jacob)/ CLOCKS_PER_SEC << std::endl;
    
    //Find shortest path
    double sum_cost;
    double temp_cost;
    double min_cost = 65536;
    int parent = -1;
    flann::Matrix<double> goal_node(seven_arm.goal_angle.data(), 1, 7);
    kd_tree.radiusSearch(goal_node, neighbors, dist, seven_arm.node_max_step, flann::SearchParams(32, 0, false));
    for(int i = 0; i < neighbors.size(); ++i){
        if(seven_arm.obstacleCollision(seven_arm.goal_angle, neighbors[0][i], obs_position)){
            sum_cost = 0;
            temp_cost = seven_arm.sumCost(neighbors[0][i], sum_cost) + (seven_arm.tree.col(neighbors[0][i]) - seven_arm.goal_angle).norm();
            if(temp_cost < min_cost){
                min_cost = temp_cost;
                parent = neighbors[0][i];
            }
        }
    }
    std::cout << "****cost****" << std::endl;
    std::cout << min_cost << std::endl;
    seven_arm.findPath(parent);
    if(!seven_arm.back_trace.empty()){
        std::cout << "Find path" << std::endl;
    }else{
        std::cout << "Can not find path" << std::endl;
        v.simStop();
        return 0;
    }

    std::cout << "****iterations****" << std::endl;
    std::cout << count << " " << "Nodes" << std::endl;
    std::cout << "****node_added****" << std::endl;
    std::cout << seven_arm.node_added << " " << "Nodes" << std::endl;
    
    //Control V-rep manipulator
    int path_ind;
    Eigen::MatrixXd joint_angle(7, 1);
    v.simStart();
    while(!seven_arm.back_trace.empty()){
        path_ind = seven_arm.back_trace.top();
        joint_angle = seven_arm.tree.col(path_ind);
        v_arm.setJointPos(joint_angle - seven_arm.start_angle);
        seven_arm.back_trace.pop();
        v.simSleep(200);
    }
    v_arm.setJointPos(seven_arm.goal_angle - seven_arm.start_angle);
    v.simSleep(1000);
    v.simStop();
    return 0;
}


