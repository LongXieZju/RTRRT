//
//  rt_rrt.cpp
//  rt_rrt
//
//  Created by 谢龙 on 2017/12/14.
//  Copyright © 2017年 long. All rights reserved.
//

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

        if(count_reroot == 60){
            break;
        }
    }
}

int main(){
    srand((unsigned)time(0));  //random sample differently every time

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

//    //test for ikine
//    v.simStart();
//    float lamda = 0.2;           // damping constant
//    float stol = 1e-3;           // tolerance
//    float nm_error = 100;        // initial error
//    int count = 0;             // iteration count
//    int ilimit = 1000;         // maximum iteration
//    Eigen::MatrixXd end_position;
//    Eigen::MatrixXd joint_position;
//    Eigen::MatrixXd error;
//    Eigen::MatrixXd J;
//    Eigen::MatrixXd partial_Jacob;
//    Eigen::MatrixXd jacob;
//    Eigen::MatrixXd jacob_inv;
//    Eigen::MatrixXd jacob_t;
//    Eigen::MatrixXd A;
//    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 1);
//    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
//    Eigen::MatrixXd f;
//    Eigen::MatrixXd a;
//    Eigen::MatrixXd b;
//    Eigen::MatrixXd I_ikine = Eigen::MatrixXd::Ones(7, 7);
//    Eigen::MatrixXd temp_matrix;
//    double q_min;
//    double q_max;
//
//    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(7, 7);
//    Eigen::MatrixXd W_star = Eigen::MatrixXd::Identity(7, 7);
//    Eigen::MatrixXd q_n = Eigen::MatrixXd::Zero(7, 1);
//    Eigen::MatrixXd q_n_star = Eigen::MatrixXd::Zero(7, 1);
//    Eigen::MatrixXd q_sns = Eigen::MatrixXd::Zero(7, 1);
//    Eigen::MatrixXd::Index maxCol;
//    Eigen::MatrixXd::Index maxRow;
//    Eigen::MatrixXd::Index minCol;
//    Eigen::MatrixXd::Index minRow;
//    Eigen::MatrixXd min_close_p;
//    Eigen::MatrixXd vector = Eigen::MatrixXd::Zero(6, 1);
//    Eigen::MatrixXd Q_max = seven_arm.max_ang;
//    Eigen::MatrixXd Q_min = seven_arm.min_ang;
//    double min_dist_ikine;
//    double s_ikine = 1;
//    int s_star = 0;
//    bool limit_exceeded = true;
//    int link_num;
//    float bound_scale;
//    double alpha = 1;
//    double gama = 0.02;
//    float length;
//    double task_scale;
//    double step_T = 0.5;
//
//    Eigen::MatrixXd joint_angle(7,1);
//    joint_angle << 0, 0, 0, M_PI/2, 0, M_PI/2, 0;
//    while(nm_error > stol){
//        q_n = Eigen::MatrixXd::Zero(7, 1);
//        W = Eigen::MatrixXd::Identity(7, 7);
//        s_ikine = 1;
//        limit_exceeded = true;
//        while(limit_exceeded){
//            limit_exceeded = false;
//            Q_max = (seven_arm.max_ang - joint_angle)/step_T;
//            Q_min = (seven_arm.min_ang - joint_angle)/step_T;
//
//
//
//            joint_position = seven_arm.fkine(joint_angle);
//            end_position = joint_position.col(3);
//            link_num = seven_arm.closestDist(joint_position, obs_position, min_close_p, min_dist_ikine);
//            std::cout << "*******min_dist_ikine********" << std::endl;
//            std::cout << min_dist_ikine << std::endl;
//            length = (min_close_p - joint_position.col(link_num - 1)).norm();
//            vector.block(0,0,3,1) = (min_close_p - joint_position.col(link_num - 1))/(min_close_p - joint_position.col(link_num - 1)).norm();
//            partial_Jacob = seven_arm.partialJacob(joint_angle, link_num, length);
//            bound_scale = 1/(1 + exp(min_dist_ikine * (2/gama) - 1) * alpha);
//            std::cout << "bound_scale" << std::endl;
//            std::cout << bound_scale << std::endl;
//
//            Eigen::MatrixXd s = partial_Jacob.transpose() * vector * min_dist_ikine;
//            for(int i = 0; i < s.rows(); i++){
//                if(s(i, 0) >= 0){
//                    Q_max = Q_max * (1 - bound_scale);
//                }else{
//                    Q_min = Q_min * (1 - bound_scale);
//                }
//            }
//
//            std::cout << "******Q_max******" << std::endl;
//            std::cout << Q_max << std::endl;
//            std::cout << "******Q_min******" << std::endl;
//            std::cout << Q_min << std::endl;
//            std::cout << "******joint_angle******" << std::endl;
//            std::cout << joint_angle << std::endl;
//
//            error = goal_position - end_position;
//            B.block(0,0,3,1) = 0.4*error;
//            J = seven_arm.jacob(joint_angle);
//            jacob = J * W;
//            jacob_t = jacob.transpose();
//            jacob_inv = (jacob_t*jacob + I_ikine * lamda).inverse()*jacob_t;
////            jacob_t = jacob.transpose();
////            A = jacob*jacob_t + lamda*lamda * I;
////            f = A.lu().solve(B * s_ikine - J * q_n);
//            q_sns = q_n + jacob_inv * (B * s_ikine - J * q_n);
//            q_min = (q_sns - Q_min).minCoeff(&minRow, &minCol);
//            q_max = (Q_max - q_sns).minCoeff(&maxRow, &maxCol);
//
//            std::cout << "******q_sns******" << std::endl;
//            std::cout << q_sns << std::endl;
//
//            if(q_min < 0 || q_max < 0){
//                limit_exceeded = true;
//                //Algorithm 2
////                a = jacob_t * A.lu().solve(B);
////                b = q_n - jacob_t * A.lu().solve(J * q_n);
//                a = jacob_inv * B;
//                b = q_n - jacob_inv * J * q_n;
//                Eigen::MatrixXd S_min(7,1);
//                Eigen::MatrixXd S_max(7,1);
//                for(int i = 0; i < seven_arm.link_num; ++i){
//                    S_min(i, 0) = (Q_min(i, 0) - b(i, 0)) / a(i, 0);
//                    S_max(i, 0) = (Q_max(i, 0) - b(i, 0)) / a(i, 0);
//                    if(S_min(i, 0) > S_max(i, 0)){
//                        double temp = S_min(i, 0);
//                        S_min(i, 0) = S_max(i, 0);
//                        S_max(i, 0) = temp;
//                    }
//                }
//                double s_max = S_max.minCoeff();
//                double s_min = S_min.maxCoeff();
//                if(s_min > s_max || s_min > 1 || s_max < 0){
//                    task_scale = 0;
//                }else{
//                    task_scale = fmin(s_max, 1);
//                }
//                //end Algorithm 2
//                if(task_scale > s_star){
//                    s_star = task_scale;
//                    W_star = W;
//                    q_n_star = q_n;
//                }
//                if(q_min < 0 && q_max < 0){
//                    if(q_min < q_max){
//                        W(minRow, minRow) = 0;
//                        q_n(minRow, 0) = Q_min(minRow, 0);
//                    }else{
//                        W(maxRow, maxRow) = 0;
//                        q_n(maxRow, 0) = Q_max(maxRow, 0);
//                    }
//                }else if(q_min < 0){
//                    W(minRow, minRow) = 0;
//                    q_n(minRow, 0) = Q_min(minRow, 0);
//                }else{
//                    W(maxRow, maxRow) = 0;
//                    q_n(maxRow, 0) = Q_max(maxRow, 0);
//                }
//                std::cout << "******q_n******" << std::endl;
//                std::cout << q_n << std::endl;
//                Eigen::FullPivLU<Eigen::MatrixXd> lu(J * W);
//                if(lu.rank() < 6){
//                    s_ikine = s_star;
//                    W = W_star;
//                    q_n = q_n_star;
//                    jacob = J * W;
//                    jacob_t = jacob.transpose();
////                    A = jacob*jacob_t + lamda*lamda * I;
////                    f = A.lu().solve(B * s_ikine - J * q_n);
//                    q_sns = q_n + (jacob_t*jacob + I_ikine * lamda).inverse() * jacob_t * (B * s_ikine - J * q_n);
//                    limit_exceeded = false;
//                }
//            }
//        }
//        joint_angle = joint_angle + q_sns;
//        v_arm.setJointPos(joint_angle - seven_arm.start_angle);
//        nm_error = error.norm();
//        count += 1;
//        if(count > ilimit){
//            std::cout<< "Solution wouldn't converge" << std::endl;
//            break;
//        }
//        v.simSleep(1000);
//    }
//    v.simStop();
//    //end test


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
    Eigen::MatrixXd joint_p;
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
            joint_p = seven_arm.fkine(new_node);
            if((joint_p.col(3) - seven_arm.goal_position).norm() < 0.01){
                std::cout << "Find a new solution" << std::endl;
                std::cout << (joint_p.col(3) - seven_arm.goal_position).norm() << std::endl;
                std::cout << new_node.transpose() << std::endl;
                v.simStart();
                v_arm.setJointPos(new_node - seven_arm.start_angle);
                v.simSleep(3000);

            }

//            kd_tree.radiusSearch(query, neighbors, dist, seven_arm.rewire_radius, flann::SearchParams(32, 0, false));
//            seven_arm.chooseParent(new_node, neighbors, nearest_node_ind, obs_position, new_node_ind);
//            seven_arm.rewire(new_node, neighbors, obs_position);

            seven_arm.insertNode(new_node, nearest_node_ind, new_node_ind);
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
    v.simStop();
    clock_t ends_jacob = clock();
    std::cout <<"RRT Running Time : "<<(double)(ends_jacob - start_jacob)/ CLOCKS_PER_SEC << std::endl;

    //Find shortest path
    double sum_cost;
    double temp_cost;
    double min_cost = INFINITY;
    int parent = -1;
    flann::Matrix<double> goal_node(seven_arm.goal_angle.data(), 1, 7);
    kd_tree.radiusSearch(goal_node, neighbors, dist, seven_arm.node_max_step, flann::SearchParams(32, 0, false));
    seven_arm.chooseParent(new_node, neighbors, neighbors[0][0], obs_position, new_node_ind);
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
    std::cout << "****cost****" << std::endl;
    std::cout << min_cost << std::endl;
    if(parent != -1){
       seven_arm.findPath(parent);
    }
    if(!seven_arm.back_trace.empty()){
        std::cout << "Find path" << std::endl;
    }else{
        std::cout << "Can not find path" << std::endl;
        v.simStop();
        return 0;
    }
    std::cout << "**** iterations ****" << std::endl;
    std::cout << count << " " << "Nodes" << std::endl;
    std::cout << "**** node_added ****" << std::endl;
    std::cout << seven_arm.node_added << " " << "Nodes" << std::endl;

    //Control V-rep manipulator
    int path_ind;
    double time_step = 0.05;
    Eigen::MatrixXd joint_angle(7, 1);
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
    obs3_vector << -1, 1, 0;
    goal_vector << 1, 0.1, 0;
//    double obs1_vel = 0.012;
//    double obs3_vel = 0.008;
//    double goal_vel = 0.002;
    double obs1_vel = 0;
    double obs3_vel = 0;
    double goal_vel = 0;
    int obh = v.getHandle("obstacle_1");

    Eigen::MatrixXd obs1_position = v.getPosition(obh);

    v.simStart();
    for(int i = 0; i < 60; ++i){
        v.setPosition(obh, obs1_position);
        v.setPosition(obstacles[2], obs3_position);
        v.setPosition(goal_handle, goal_position);
        obs_position.col(0) = obs1_position;
        obs_position.col(2) = obs3_position;

        seven_arm.goal_angle = seven_arm.ikine(goal_position);

        if(!seven_arm.back_trace.empty()){
            path_ind = seven_arm.back_trace.top();
            std::cout << "**** root ****" << std::endl;
            std::cout << "root_index: " << path_ind << std::endl;
            joint_angle = seven_arm.tree.col(path_ind);
            v_arm.setJointPos(joint_angle - seven_arm.start_angle);
            seven_arm.back_trace.pop();
            v.simSleep(500);

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
                seven_arm.findPath(parent);
            }
        }else{
            v_arm.setJointPos(seven_arm.goal_angle - seven_arm.start_angle);
            v.simSleep(800);
//            break;
        }
        obs1_position = v.getPosition(obh) + obs1_vel * time_step * obs1_vector / obs1_vector.norm() * i;
        obs3_position = v.getPosition(obstacles[2]) + obs3_vel * time_step * obs3_vector / obs3_vector.norm() * i;
        goal_position = v.getPosition(goal_handle) + goal_vel * time_step * goal_vector / goal_vector.norm() * i;
    }
    v.simSleep(10000);
    v.simStop();

    return 0;
}





