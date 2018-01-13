//
//  rt_rrt.cpp
//  rt_rrt
//
//  Created by 谢龙 on 2017/12/14.
//  Copyright © 2017年 long. All rights reserved.
//

#include <unistd.h>
#include <sys/time.h>
#include <time.h>
#include <iostream>
#include <math.h>
#include <queue>
#include <set>
#include <eigen3/Eigen/Geometry>
#include <fstream>
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
//
void rootRewire(int root_node, Manipulator& seven_arm, flann::Index<flann::L2<double> >& kd_tree, Eigen::MatrixXd& obs_position){
    std::queue<int> node_index;
    std::set<int> node_has;
    node_index.push(root_node);
    node_has.insert(root_node);
    std::vector<std::vector<int> > index;
    std::vector<std::vector<int> > neighbors;
    std::vector<std::vector<double> > dist;
    Eigen::MatrixXd new_node(seven_arm.link_num,1);
    
    
    int root;
    double old_cost;
    double new_cost;
    int count_reroot = 0;
    
    seven_arm.parent(0, seven_arm.root_node) = root_node;
    seven_arm.sum_cost(0, root_node) = 0;
    seven_arm.parent(0, root_node) = root_node;
    seven_arm.root_node = root_node;
    while(!node_index.empty()){
        root = node_index.front();
        node_index.pop();
        flann::Matrix<double> node(seven_arm.tree.col(root).data(), 1, 7);
        kd_tree.radiusSearch(node, neighbors, dist, seven_arm.rewire_radius, flann::SearchParams(32, 0, false));
        count_reroot ++;
        //        std::cout << "****** count ********" << std::endl;
        //        std::cout << count_reroot << std::endl;
        //        std::cout << "****** neighbor_size ********" << std::endl;
        //        std::cout << neighbors[0].size() << std::endl;
        for(int i = 0; i < neighbors[0].size(); ++i){
            if(!node_has.count(neighbors[0][i])){
                node_index.push(neighbors[0][i]);
                node_has.insert(neighbors[0][i]);
            }
            new_node = seven_arm.tree.col(root);
            if(seven_arm.obstacleCollision(new_node, neighbors[0][i], obs_position)){
                old_cost = 0;
                old_cost = seven_arm.sumCost(neighbors[0][i], old_cost);
                new_cost = seven_arm.sum_cost(0, root) + (seven_arm.tree.col(neighbors[0][i]) - seven_arm.tree.col(root)).norm();
                if(new_cost < old_cost){
                    seven_arm.sum_cost(0, neighbors[0][i]) = new_cost;
                    seven_arm.parent(0, neighbors[0][i]) = root;
                }
            }
        }
        
        if(count_reroot == 100){
            break;
        }
    }
}

/*****************************
 *****************************
 *****************************
 *****************************
 *****************************
 *****************************/

int main(){
    srand((unsigned)time(0));  //random sample differently every time
    
    std::cout << (double)rand()/RAND_MAX << std::endl;
    std::string file_path = "/Users/xielong/Desktop/save_data/online_1_13_01_13/";
    
//    //Set time to run other code
//    struct timeval begin_t; //(1)
//    struct timeval end_t;
//    gettimeofday(&begin_t, NULL);
//    unsigned int BEGIN = 0;
//    printf("time %u:%u\n", begin_t.tv_sec, begin_t.tv_usec);
//    while(true){
//        gettimeofday(&end_t, NULL); //(2)
//        BEGIN = (end_t.tv_sec - begin_t.tv_sec) * 1000000 + (end_t.tv_usec - begin_t.tv_usec);
//        if(BEGIN == 5 * 1000000){
//            printf("time %u:%u\n", end_t.tv_sec, end_t.tv_usec);
//        }
//    }
//    //Succeed!
    
    // Manipulator model, contains forward and backward kinematics models
    double min_cost = INFINITY;
    int parent = -1;
    double sum_cost;
    double temp_cost;
    
    
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
    //    v.simStop();
    v.connect();
    int goal_handle = v.getHandle("goal");
    Eigen::MatrixXd goal_position = v.getPosition(goal_handle);
    std::cout << "****goal_position****" << std::endl;
    std::cout << goal_position.transpose() << std::endl;
    int obstacles[3] = {v.getHandle("obstacle_1"), v.getHandle("obstacle_2"), v.getHandle("obstacle_3")};
    std::cout << "****obs_handle****" << std::endl;
    std::cout << obstacles[0] << "," << obstacles[1] << "," << obstacles[2] << std::endl;
    Eigen::MatrixXd obs_position = getObstaclesPos(obstacles, v);
    
    //KD-tree
    std::vector<std::vector<int> > index;
    std::vector<std::vector<int> > neighbors;
    std::vector<std::vector<double> > dist;
    flann::Matrix<double> point(joint_start.data(), 1, seven_arm.link_num);
    flann::Index<flann::L2<double>> kd_tree(point, flann::KDTreeIndexParams(4));
    kd_tree.buildIndex();
    
    // VREP manipulator model
    VREP_arm v_arm("redundantRob", v.clientID, v.mode);
        
    while(min_cost > 2){
        // Setting up
        flann::Matrix<double> point(joint_start.data(), 1, seven_arm.link_num);
        flann::Index<flann::L2<double>> kd_tree(point, flann::KDTreeIndexParams(4));
        kd_tree.buildIndex();
        seven_arm.setGoalPosition(goal_position);
        seven_arm.setStartState(joint_start);
        std::cout << "****goal_angle****" << std::endl;
        std::cout << seven_arm.goal_angle.transpose() << std::endl;
        std::cout << "****start_angle****" << std::endl;
        std::cout << seven_arm.start_angle.transpose() << std::endl;
        std::cout << "****obs_position****" << std::endl;
        std::cout << obs_position.transpose() << std::endl;
        
        //Save obstacles position
        std::ofstream obs_out(file_path + "obs_position.txt", std::ios::out | std::ios::trunc);
        if (obs_out.is_open())
        {
            obs_out << obs_position.transpose() << "\n";
            obs_out.close();
            std::cout << "Success store the obstacles' position!" << std::endl;
        }
        
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
            min_dist = sqrt(dist[0][0]);
            if(min_dist  > seven_arm.node_max_step){
                seven_arm.steer(new_node, nearest_node_ind);
            }
            if(seven_arm.obstacleCollision(new_node, nearest_node_ind, obs_position)){
                flann::Matrix<double> query(new_node.data(), 1, 7);
                
                //RRT*
                kd_tree.radiusSearch(query, neighbors, dist, seven_arm.rewire_radius, flann::SearchParams(32, 0, false));
                seven_arm.chooseParent(new_node, neighbors, nearest_node_ind, obs_position, new_node_ind);
                seven_arm.rewire(new_node, neighbors, obs_position);
                
    //            seven_arm.insertNode(new_node, nearest_node_ind, new_node_ind);
                flann::Matrix<double> node(seven_arm.tree.col(new_node_ind).data(), 1, 7);
                kd_tree.addPoints(node);
                //            if((new_node - seven_arm.goal_angle).norm() < seven_arm.node_max_step){
                //                if(seven_arm.obstacleCollision(new_node, seven_arm.goal_angle, obs_position)){
                //                    seven_arm.findPath(new_node_ind);
                //                    double cost = 0;
                //                    seven_arm.sumCost(new_node_ind, cost);
                //                    cost = cost + (seven_arm.tree.col(new_node_ind) - seven_arm.goal_angle).norm();
                //                    std::cout << "**************" << std::endl;
                //                    std::cout << "Find path: " << cost << std::endl;
                //                    std::stack<int>().swap(seven_arm.back_trace);
                //                    break;
                //                }
                //            }
            }
        }
        clock_t ends_jacob = clock();
        std::cout <<"RRT Running Time : "<<(double)(ends_jacob - start_jacob)/ CLOCKS_PER_SEC << std::endl;
        
        //Save the tree
        std::ofstream tree_out(file_path + "tree.txt", std::ios::out | std::ios::trunc);
        if (tree_out.is_open())
        {
            tree_out << seven_arm.tree << "\n";
            tree_out.close();
            std::cout << "Success store tree!" << std::endl;
        }
        
        //Find shortest path
//        double sum_cost;
//        double temp_cost;
        
//        int parent = -1;
        flann::Matrix<double> goal_node(seven_arm.goal_angle.data(), 1, 7);
        kd_tree.radiusSearch(goal_node, neighbors, dist, seven_arm.node_max_step, flann::SearchParams(32, 0, false));
        std::cout << "nearest distance:" << dist[0][0] << std::endl;
        if(neighbors[0].size() == 0){
            std::cout << "no near nodes" << std::endl;
            continue;
//            return 0;
        }
        seven_arm.chooseParent(new_node, neighbors, neighbors[0][0], obs_position, new_node_ind);
        //    seven_arm.rewire(seven_arm.goal_angle, neighbors, obs_position);
        
        for(int i = 0; i < neighbors[0].size(); ++i){
            if(seven_arm.obstacleCollision(seven_arm.goal_angle, neighbors[0][i], obs_position)){
                sum_cost = 0;
                temp_cost = seven_arm.sumCost(neighbors[0][i], sum_cost) + (seven_arm.tree.col(neighbors[0][i]) - seven_arm.goal_angle).norm();
                if(temp_cost < min_cost){
                    min_cost = temp_cost;
                    parent = neighbors[0][i];
                }
            }
            //        std::cout << "****cost****" << std::endl;
            //        std::cout << "min_cost: " << min_cost << std::endl;
        }
        std::cout << "****cost****" << std::endl;
        std::cout << min_cost << std::endl;
//        if(min_cost > 2){
//            return 0;
//        }
        if(parent != -1){
            seven_arm.findPath(parent);
        }
        if(!seven_arm.back_trace.empty()){
            std::cout << "Find path" << std::endl;
        }else{
            std::cout << "Can not find path" << std::endl;
            v.simStop();
            continue;
//            return 0;
        }
        std::cout << "**** iterations ****" << std::endl;
        std::cout << count << " " << "Nodes" << std::endl;
        std::cout << "**** node_added ****" << std::endl;
        std::cout << seven_arm.node_added << " " << "Nodes" << std::endl;
    }
    
//    double sum_cost;
//    double temp_cost;
    
    //Control V-rep manipulator
    int path_ind;
    double time_step = 0.05;
    Eigen::MatrixXd joint_angle(7, 1);
    Eigen::MatrixXd current_joint_angle(7, 1);
    //    Eigen::MatrixXd obs1_position = v.getPosition(obstacles[0]);
    Eigen::MatrixXd obs2_position = v.getPosition(obstacles[1]);
    Eigen::MatrixXd obs3_position = v.getPosition(obstacles[2]);
    //    Eigen::MatrixXd goal_position = v.getPosition('goal');
    //    double obs1_vel = 0.1;
    //    double obs3_vel = 0.01;
    Eigen::MatrixXd obs1_vector(3, 1);
    Eigen::MatrixXd obs3_vector(3, 1);
    Eigen::MatrixXd goal_vector(3, 1);
    //    obs1_vector << -1, 0, 0.2;
    //    obs3_vector << 0, 0, 0;
    
    obs1_vector << -1, 0, 0;
    obs3_vector << 0, 0, 1;
    goal_vector << 1, 0.1, 0;
//    double obs1_vel = 0.017;
//    double obs3_vel = 0.08;
//    double goal_vel = 0.000;
    
    double obs1_vel = 0.00;
    double obs3_vel = 0.00;
    double goal_vel = 0.000;
    int obh = v.getHandle("obstacle_1");
    
    Eigen::MatrixXd obs1_position = v.getPosition(obh);
    std::ofstream path_out(file_path + "path.txt", std::ios::out | std::ios::trunc);
    
    struct timeval begin_t; //(1)
    struct timeval end_t;
    gettimeofday(&begin_t, NULL);
    long BEGIN = 0;
    bool flag = true;
    
    v.simStart();
    clock_t start_motion = clock();
    for(int i = 0; i < 60; ++i){
        v.setPosition(obh, obs1_position);
        v.setPosition(obstacles[2], obs3_position);
        v.setPosition(goal_handle, goal_position);
        obs_position.col(0) = obs1_position;
        obs_position.col(2) = obs3_position;
        
        //        seven_arm.goal_angle = seven_arm.ikine(goal_position);
        seven_arm.goal_angle = seven_arm.ikineStart(goal_position, seven_arm.goal_angle);
        
        //Set time to run other code
        flag = true;
        gettimeofday(&begin_t, NULL);
        while(flag){
            gettimeofday(&end_t, NULL); //(2)
            BEGIN = (end_t.tv_sec - begin_t.tv_sec) * 1000000 + (end_t.tv_usec - begin_t.tv_usec);
            if(BEGIN >= 0.2 * 1000000){
                flag = false;
//                printf("[%ld]\n", BEGIN/1000000);
            }else{
                if(!seven_arm.back_trace.empty()){
                    rootRewire(seven_arm.back_trace.top(), seven_arm, kd_tree, obs_position);
                    for(int i = 0; i < neighbors[0].size(); ++i){
                        if(seven_arm.obstacleCollision(seven_arm.goal_angle, neighbors[0][i], obs_position)){
                            sum_cost = 0;
                            temp_cost = seven_arm.sumCost(neighbors[0][i], sum_cost) + (seven_arm.tree.col(neighbors[0][i]) - seven_arm.goal_angle).norm();
                            if(temp_cost < min_cost){
                                min_cost = temp_cost;
                                parent = neighbors[0][i];
                            }
                        }
                    }
//                    std::cout << "********parent**********" << std::endl;
//                    std::cout << parent << std::endl;
                    seven_arm.findPath(parent);
                }
                
                if(!seven_arm.obstacleCollision(seven_arm.goal_angle, seven_arm.goal_angle, obs_position)){
//                    std::cout << "Can't reach the goal" << std::endl;
//                    std::cout << "Obstacles position : " << obs_position << std::endl;
                    //            seven_arm.goal_angle << 0.3983, 1.1493, 1.8270, 1.3542, -1.1668, 1.8637, -0.2898;
                    //            seven_arm.goal_angle << 0.3684, 0.8282, 1.4708, 1.3542, -0.7964, 1.6225, -0.2898;
                    seven_arm.goal_angle = seven_arm.goal_angle_2;
//                    std::cout << "Change the goal to : "<< "\n" << seven_arm.goal_angle << std::endl;
                    obs3_vel = 0.008;
                    
                    flann::Matrix<double> goal_node(seven_arm.goal_angle.data(), 1, 7);
                    kd_tree.radiusSearch(goal_node, neighbors, dist, 100, flann::SearchParams(32, 0, false));
//                    std::cout << "nearest distance change:" << dist[0][0] << std::endl;
                    min_cost = INFINITY;
                    for(int i = 0; i < neighbors[0].size(); ++i){
                        if(seven_arm.obstacleCollision(seven_arm.goal_angle, neighbors[0][i], obs_position)){
                            sum_cost = 0;
                            temp_cost = seven_arm.sumCost(neighbors[0][i], sum_cost) + (seven_arm.tree.col(neighbors[0][i]) - seven_arm.goal_angle).norm();
                            if(temp_cost < min_cost){
                                min_cost = temp_cost;
                                parent = neighbors[0][i];
                            }
                        }
                    }
//                    std::cout << parent << std::endl;
                    //            seven_arm.findPath(parent);
                    if(neighbors[0].size() == 0){
                        std::cout << "no near nodes change" << std::endl;
                        return 0;
                    }
                }
            }
        }
        
        //Save the path
        current_joint_angle = v_arm.getJointAngle();
        if (path_out.is_open()){
            path_out << current_joint_angle.transpose() << "\n";
        }
        if(i == 0){
            if(!seven_arm.back_trace.empty()){
                path_ind = seven_arm.back_trace.top();
//                std::cout << "**** root ****" << std::endl;
//                std::cout << "root_index: " << path_ind << std::endl;
                
                joint_angle = seven_arm.tree.col(path_ind);
                v_arm.setJointPos(joint_angle - seven_arm.start_angle);
//                seven_arm.back_trace.pop();
            }else{
                v_arm.setJointPos(seven_arm.goal_angle - seven_arm.start_angle);
            }
        }
        
        current_joint_angle = current_joint_angle + seven_arm.start_angle;
//        std::cout << "**** current_joint_angle ****" << std::endl;
//        std::cout << "current_joint_angle: " << current_joint_angle << std::endl;
//
//        std::cout << "**** back_trace ****" << std::endl;
//        std::cout << "current_joint_angle: " << seven_arm.tree.col(seven_arm.back_trace.top()) << std::endl;
//
//        std::cout << "**** norm ****" << std::endl;
//        std::cout << "norm: " << (current_joint_angle - seven_arm.tree.col(seven_arm.back_trace.top())).norm() << std::endl;
        if(!seven_arm.back_trace.empty()){
            if((current_joint_angle - seven_arm.tree.col(seven_arm.back_trace.top())).norm() < 0.05){
                if(!seven_arm.back_trace.empty()){
                    seven_arm.back_trace.pop();
                }
                if(!seven_arm.back_trace.empty()){
                    path_ind = seven_arm.back_trace.top();
    //                std::cout << "**** root ****" << std::endl;
    //                std::cout << "root_index: " << path_ind << std::endl;
                    
                    joint_angle = seven_arm.tree.col(path_ind);
                    v_arm.setJointPos(joint_angle - seven_arm.start_angle);
    //                seven_arm.back_trace.pop();
                }else{
                    v_arm.setJointPos(seven_arm.goal_angle - seven_arm.start_angle);
                }
            }
        }
        
        obs1_position = v.getPosition(obh) + obs1_vel * time_step * obs1_vector / obs1_vector.norm() * i;
        obs3_position = v.getPosition(obstacles[2]) + obs3_vel * time_step * obs3_vector / obs3_vector.norm() * i;
        goal_position = v.getPosition(goal_handle) + goal_vel * time_step * goal_vector / goal_vector.norm() * i;
//        v.simSleep(500);
//        sleep(0.5);
    }
    clock_t ends_motion = clock();
    std::cout <<"Motion Time : "<<(double)(ends_motion - start_motion)/ CLOCKS_PER_SEC << std::endl;
    v.simSleep(1000);
    v.simStop();
    path_out.close();
    std::cout << "Success store the path!" << std::endl;
    
    return 0;
}
