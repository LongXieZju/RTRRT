//
//  Manipulator.hpp
//  rt_rrt
//
//  Created by 谢龙 on 2017/12/14.
//  Copyright © 2017年 long. All rights reserved.
//

#ifndef Manipulator_hpp
#define Manipulator_hpp

#include <stdio.h>
#include "nodeStruct.h"
#include <flann/flann.hpp>
//#include <flann/io/hdf5.h>
#include <stack>

#define Eigen2Flann Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>

class Manipulator{
private:
    Eigen::MatrixXd joint_angle;
    Eigen::MatrixXd dh_param;
    Eigen::MatrixXd T[4];
    
public:
    int link_num;
    int max_iter;
    int step_div;
    int node_added;
    int obstacle_num;
    int root_node;
    int goal_node;
    double rewire_radius;
    float arm_radius;
    float goal_bais;
    float node_max_step;
    float obs_radius[3] = {0.08, 0.1, 0.07}; // +0.02
    //    float obs_radius[3] = {0.11, 0.2, 0.07};
    float link_length[3] = {0.29126, 0.32363, 0.15512};
    std::stack<int> back_trace;
    
    Eigen::MatrixXd goal_position;
    Eigen::MatrixXd goal_angle;
    Eigen::MatrixXd start_angle;
    Eigen::MatrixXd max_ang;
    Eigen::MatrixXd min_ang;
    
    Eigen::MatrixXd tree;
    Eigen::MatrixXd parent;
    Eigen::MatrixXd children;
    Eigen::MatrixXd sum_cost;
    
    flann::Index<flann::L2<double> >* kd_tree;
public:
    Manipulator(Eigen::MatrixXd dh_param);
    void setJointAngle(Eigen::MatrixXd q);
    void setDhParam(Eigen::MatrixXd dh_param);
    Eigen::MatrixXd getDhParam();
    void setGoalPosition(Eigen::MatrixXd goal_position);
    void setStartState(Eigen::MatrixXd joint_angle);
    
    
    Eigen::MatrixXd jacob(Eigen::MatrixXd joint_angle);
    Eigen::MatrixXd fkine(Eigen::MatrixXd joint_angle);
    Eigen::MatrixXd ikine(Eigen::MatrixXd end_effector);
    
    Eigen::Matrix4Xd transMatrix(Eigen::MatrixXd manipulator_dh, float q);
    
    Eigen::MatrixXd sampleNode();
//    Eigen::MatrixXd steer(Eigen::MatrixXd new_node, int nearest_node_ind);
    void steer(Eigen::MatrixXd& new_node, int& nearest_node_ind);
    void getNearestNode(Eigen::MatrixXd node, NearestNode* nearest_node);
    void getNearestNode_kd(Eigen::MatrixXd& node, NearestNode& nearest_node);
    Eigen::MatrixXd getNeighbors(Eigen::MatrixXd& new_node, int& nearest_node_ind, Eigen::MatrixXd& neighbor_nodes);
    //    int obstacleCollision(Eigen::MatrixXd new_node, int nearest_node_ind, Eigen::MatrixXd obs_position);
    int obstacleCollision(Eigen::MatrixXd& new_node, int& nearest_node_ind, Eigen::MatrixXd& obs_position);
    int obstacleCollision(Eigen::MatrixXd& new_node, Eigen::MatrixXd& goal_node, Eigen::MatrixXd& obs_position);
    //    float linkObstacleCollision(Eigen::MatrixXd P1, Eigen::MatrixXd P2, Eigen::MatrixXd obstacle);
    void insertNode(Eigen::MatrixXd& new_node, int& nearest_node_ind, int& new_node_ind);
    void findPath(int new_node_ind);
    
    void chooseParent(Eigen::MatrixXd& new_node, std::vector<std::vector<int> >& neighbor_nodes,
                      int& nearest_node_ind, Eigen::MatrixXd& obs_position, int& new_node_ind);
    void rewire(Eigen::MatrixXd& new_node, std::vector<std::vector<int> >& neighbor_nodes, Eigen::MatrixXd& obs_position);
    double sumCost(int& nearest_node_ind, double sum_cost);
};

#endif /* Manipulator_hpp */
