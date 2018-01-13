//
//  Manipulator.cpp
//  rt_rrt
//
//  Created by 谢龙 on 2017/12/14.
//  Copyright © 2017年 long. All rights reserved.
//

#include "Manipulator.hpp"
#include <iostream>
#include <math.h>
//#include <flann/flann.hpp>
#include <eigen3/Eigen/Geometry>

Manipulator::Manipulator(Eigen::MatrixXd dh_param){
    Eigen::MatrixXd q_init = Eigen::MatrixXd::Zero(7,1);
    setJointAngle(q_init);
    setDhParam(dh_param);

    Manipulator::goal_bais = 0.03;
    Manipulator::link_num = 7;
    Manipulator::max_iter = 10000 * 2;
    Manipulator::step_div = 2;
    Manipulator::obstacle_num = 3;
//    Manipulator::node_max_step = 0.0462; // sqrt(sum(((1 * pi / 180)*ones(7, 1)).^2))
    Manipulator::node_max_step = 0.1;

    Manipulator::max_ang = 130*M_PI/180 * Eigen::MatrixXd::Ones(Manipulator::link_num, 1);
    Manipulator::min_ang = -90*M_PI/180 * Eigen::MatrixXd::Ones(Manipulator::link_num, 1);
    Manipulator::arm_radius = 0.04;
    Manipulator::rewire_radius = 0.05;

    Manipulator::tree = Eigen::MatrixXd::Zero(Manipulator::link_num, Manipulator::max_iter);
    Manipulator::parent = Eigen::MatrixXd::Zero(1, Manipulator::max_iter);
    Manipulator::children = Eigen::MatrixXd::Zero(1, Manipulator::max_iter);
    Manipulator::sum_cost = Eigen::MatrixXd::Zero(1, Manipulator::max_iter);
}

void Manipulator::setJointAngle(Eigen::MatrixXd joint_angle){
    Manipulator::joint_angle = joint_angle;
}

void Manipulator::setDhParam(Eigen::MatrixXd dh_param){
    Manipulator::dh_param = dh_param;
}

Eigen::MatrixXd Manipulator::getDhParam(){
    return Manipulator::dh_param;
}

void Manipulator::setGoalPosition(Eigen::MatrixXd goal_position){
    Manipulator::goal_position = goal_position;
    Manipulator::goal_angle = Manipulator::ikine(goal_position);
//    Manipulator::goal_angle << 1.1361, 0.3742, 0.0850, 1.3542, 0.0052, 1.4092, -0.2898;
    Manipulator::goal_angle << 1.1669, 0.3735, 0.0434, 1.3542, 0.0206, 1.4093, -0.3668;
    Manipulator::goal_angle_2 = Manipulator::ikine(goal_position);
//    Manipulator::goal_angle_2 << 0.3983, 1.1493, 1.8270, -1.7874, -1.1668, -1.2779, -0.2898;
    Manipulator::goal_angle_2 << 0.3684, 0.8282, 1.4708, 1.3542, -0.7964, 1.6225, -0.2898;
//    Manipulator::goal_angle_2 << 2.0223, 0.7854, -1.4162, 1.3542, 0.8088, 1.6292, -0.2898;
    
    
}

void Manipulator::setStartState(Eigen::MatrixXd joint_angle){
    Manipulator::start_angle = joint_angle;
    Manipulator::tree.col(0) = joint_angle;
    Manipulator::node_added = 1;
    Manipulator::root_node = 0;
}

Eigen::MatrixXd Manipulator::jacob(Eigen::MatrixXd joint_angle){
    Eigen::MatrixXd Jn = Eigen::MatrixXd::Zero(6, 7);
    Eigen::MatrixXd J0 = Eigen::MatrixXd::Zero(6, 7);
    Eigen::MatrixXd J_temp = Eigen::MatrixXd::Zero(6, 6);
    Eigen::MatrixXd UT = Eigen::MatrixXd::Identity(4, 4);
    Eigen::MatrixXd dh = Manipulator::getDhParam();
    Eigen::Matrix4Xd T_temp(4,4);
    Eigen::MatrixXd delta(3, 1);
    Eigen::MatrixXd D(3, 1);
    for(int i = 6; i >= 0; i--){
        float q = joint_angle(i);
        float d = dh(i,1);
        float a = dh(i,2);
        float alpha = dh(i,3);
        T_temp << cos(q), -sin(q)*cos(alpha), sin(q)*sin(alpha), a*cos(q),
        sin(q),  cos(q)*cos(alpha),-cos(q)*sin(alpha), a*sin(q),
        0,         sin(alpha),         cos(alpha),       d,
        0,            0,                 0,              1;
        UT = T_temp*UT;
        D << -UT(0,0)*UT(1,3)+UT(1,0)*UT(0,3),
        -UT(0,1)*UT(1,3)+UT(1,1)*UT(0,3),
        -UT(0,2)*UT(1,3)+UT(1,2)*UT(0,3);
        delta = UT.block(2,0,1,3).transpose();
        Jn.block(0,i,3,1) = D;
        Jn.block(3,i,3,1) = delta;
    }
    J_temp.block(0,0,3,3) = UT.block(0,0,3,3);
    J_temp.block(3,3,3,3) = UT.block(0,0,3,3);
    J0 = J_temp * Jn;
    return J0;
}

Eigen::MatrixXd Manipulator::fkine(Eigen::MatrixXd joint_angle){
    Eigen::MatrixXd joint_position(3,4);
    joint_position.col(0) << 0, 0, 0.20386;
    Eigen::MatrixXd dh = Manipulator::getDhParam();
    Eigen::Matrix4Xd T = Eigen::Matrix4Xd::Identity(4,4);
    for(int i = 0; i < joint_angle.rows(); i++){
        float q = joint_angle(i);
        float d = dh(i,1);
        float a = dh(i,2);
        float alpha = dh(i,3);
        Eigen::Matrix4Xd T_temp(4,4);
        T_temp << cos(q), -sin(q)*cos(alpha), sin(q)*sin(alpha), a*cos(q),
        sin(q),  cos(q)*cos(alpha),-cos(q)*sin(alpha), a*sin(q),
        0,         sin(alpha),         cos(alpha),       d,
        0,            0,                 0,              1;
        T = T*T_temp;
        if(i == 3){
            joint_position.col(1) = T.block(0,3,3,1);
        }
        if(i == 5){
            joint_position.col(2) = T.block(0,3,3,1);
        }
    }
    joint_position.col(3) = T.block(0,3,3,1);
    return joint_position;
}

Eigen::MatrixXd Manipulator::ikine(Eigen::MatrixXd goal_position){
    float lamda = 0.2;           // damping constant
    float stol = 1e-3;           // tolerance
    float nm_error = 100;        // initial error
    int count = 0;             // iteration count
    int ilimit = 1000;         // maximum iteration
    Eigen::MatrixXd end_position;
    Eigen::MatrixXd error;
    Eigen::MatrixXd jacob;
    Eigen::MatrixXd jacob_t;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 1);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd f;

    Eigen::MatrixXd joint_angle(7,1);
    joint_angle << 0, 0, 0, M_PI/2, 0, M_PI/2, 0;
    while(nm_error > stol){
        end_position = Manipulator::fkine(joint_angle).col(3);
        error = goal_position - end_position;
        B.block(0,0,3,1) = error;
        jacob = Manipulator::jacob(joint_angle);
        jacob_t = jacob.transpose();
        A = jacob*jacob_t + lamda*lamda * I;
        f = A.lu().solve(B);
        joint_angle = joint_angle + jacob_t * f;
        nm_error = error.norm();
        count += 1;
        if(count > ilimit){
            std::cout<< "Solution wouldn't converge" << std::endl;
            break;
        }
    }
    return joint_angle;
}

Eigen::MatrixXd Manipulator::ikineStart(Eigen::MatrixXd goal_position, Eigen::MatrixXd joint_angle){
    float lamda = 0.2;           // damping constant
    float stol = 1e-3;           // tolerance
    float nm_error = 100;        // initial error
    int count = 0;             // iteration count
    int ilimit = 1000;         // maximum iteration
    Eigen::MatrixXd end_position;
    Eigen::MatrixXd error;
    Eigen::MatrixXd jacob;
    Eigen::MatrixXd jacob_t;
    Eigen::MatrixXd A;
    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 1);
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd f;
    
    //    Eigen::MatrixXd joint_angle(7,1);
    //    joint_angle << 0, 0, 0, M_PI/2, 0, M_PI/2, 0;
    while(nm_error > stol){
        end_position = Manipulator::fkine(joint_angle).col(3);
        error = goal_position - end_position;
        B.block(0,0,3,1) = error;
        jacob = Manipulator::jacob(joint_angle);
        jacob_t = jacob.transpose();
        A = jacob*jacob_t + lamda*lamda * I;
        f = A.lu().solve(B);
        joint_angle = joint_angle + jacob_t * f;
        nm_error = error.norm();
        count += 1;
        if(count > ilimit){
            std::cout<< "Solution wouldn't converge" << std::endl;
            break;
        }
    }
    return joint_angle;
}

Eigen::Matrix4Xd Manipulator::transMatrix (Eigen::MatrixXd link, float q){
    float d = link(1);
    float a = link(2);
    float alpha = link(3);
    Eigen::Matrix4Xd result(4,4);
    result << cos(q), -sin(q)*cos(alpha), sin(q)*sin(alpha), a*cos(q),
    sin(q),  cos(q)*cos(alpha),-cos(q)*sin(alpha), a*sin(q),
    0,         sin(alpha),         cos(alpha),       d,
    0,            0,                 0,              1;
    return result;
}

Eigen::MatrixXd Manipulator::sampleNode(){
    Eigen::MatrixXd state;
    if((double)rand()/RAND_MAX < Manipulator::goal_bais){
        if((double)rand()/RAND_MAX < 0.3){
            state = Manipulator::goal_angle + Eigen::MatrixXd::Random(Manipulator::link_num, 1) * Manipulator::node_max_step;
        }else{
            state = Manipulator::goal_angle + Eigen::MatrixXd::Random(Manipulator::link_num, 1) * Manipulator::node_max_step;
        }
    }else{
        Eigen::MatrixXd step = Manipulator::max_ang - Manipulator::min_ang;
        Eigen::MatrixXd random = (Eigen::MatrixXd::Random(Manipulator::link_num, 1)
                                  + Eigen::MatrixXd::Ones(Manipulator::link_num, 1)) / 2;
        state = step.cwiseProduct(random) +  Manipulator::min_ang;
    }
    return state;
}

Eigen::MatrixXd Manipulator::sampleNode(Eigen::MatrixXd goal_node){
    Eigen::MatrixXd state;
    if((double)rand()/RAND_MAX < 0.8){
        state = goal_node + Eigen::MatrixXd::Random(Manipulator::link_num, 1) * Manipulator::node_max_step;
    }else{
        Eigen::MatrixXd step = Manipulator::max_ang - Manipulator::min_ang;
        Eigen::MatrixXd random = (Eigen::MatrixXd::Random(Manipulator::link_num, 1)
                                  + Eigen::MatrixXd::Ones(Manipulator::link_num, 1)) / 2;
        state = step.cwiseProduct(random) +  Manipulator::min_ang;
    }
    return state;
}

void Manipulator::getNearestNode(Eigen::MatrixXd node, NearestNode* nearest_node){
    float dist = 65536;
    int index = 0;
    for(int i = 0; i < Manipulator::node_added; i++){
        float dist_temp = (Manipulator::tree.col(i) - node).norm();
        if(dist > dist_temp){
            dist = dist_temp;
            index = i;
        }
    }
    nearest_node->ind = index;
    nearest_node->nearest_dist = dist;
}

void Manipulator::getNearestNode_kd(Eigen::MatrixXd& node, NearestNode& nearest_node){
    std::vector<std::vector<int> > index;
    std::vector<std::vector<double> > dist;
    flann::Matrix<double> query_node(node.data(), 1, 7);
    Manipulator::kd_tree->knnSearch(query_node, index, dist, 1, flann::SearchParams(32, 0, false));
    nearest_node.ind = index[0][0];
    nearest_node.nearest_dist = dist[0][0];
}

Eigen::MatrixXd Manipulator::getNeighbors(Eigen::MatrixXd& new_node, int& nearest_node_ind, Eigen::MatrixXd& neighbor_nodes){

    Eigen::MatrixXd from;
    return from;
}

void Manipulator::steer(Eigen::MatrixXd& new_node, int& nearest_node_ind){
    Eigen::MatrixXd from = Manipulator::tree.col(nearest_node_ind);
    Eigen::MatrixXd angle_diff = new_node - from;
    new_node = from + angle_diff / angle_diff.norm() * Manipulator::node_max_step;
}

int Manipulator::obstacleCollision(Eigen::MatrixXd& new_node, int& nearest_node_ind, Eigen::MatrixXd& obs_position){
    Eigen::MatrixXd nearest_node = Manipulator::tree.col(nearest_node_ind);
    Eigen::MatrixXd dist_temp = new_node - nearest_node;
    double dist = dist_temp.norm();
    Eigen::MatrixXd vector = dist_temp/dist;
    double step = dist / Manipulator::step_div;

    Eigen::MatrixXd state_angle;
    Eigen::MatrixXd joint_position;

    Eigen::MatrixXd obs_dist;
    for(int i = 0; i < Manipulator::obstacle_num; ++i){
        for(int j = 1; j <= Manipulator::step_div; ++j){
            state_angle = nearest_node + j*step*vector;
            joint_position = Manipulator::fkine(state_angle);
            obs_dist = (joint_position - obs_position.col(i).replicate(1, 4)).colwise().norm();
            float dist = 65536;
            for(int k = 0; k < 3; ++k){
                if(obs_dist(0, k) + obs_dist(0, k+1) == Manipulator::link_length[k]){
                    return 0;
                }else if(pow(obs_dist(0, k+1), 2) >= pow(obs_dist(0, k), 2) + pow(Manipulator::link_length[k], 2)){
                    if(obs_dist(0, k) < dist){
                        dist = obs_dist(0, k);
                    }
                }else if(pow(obs_dist(0, k), 2) >= pow(obs_dist(0, k+1), 2) + pow(Manipulator::link_length[k], 2)){
                    if(obs_dist(0, k+1) < dist){
                        dist = obs_dist(0, k+1);
                    }
                }else{
                    float p = (obs_dist(0, k) + obs_dist(0, k+1) + Manipulator::link_length[k])/2;
                    float S = sqrt(p*(p - obs_dist(0, k))*(p - obs_dist(0, k+1))*(p - Manipulator::link_length[k]));
                    float h = 2*S/Manipulator::link_length[k];
                    if(h < dist){
                        dist = h;
                    }
                }
            }
            dist = dist - Manipulator::arm_radius - Manipulator::obs_radius[i];
            if(dist < 0){
                return 0;
            }
        }
    }
    return 1;
}

int Manipulator::obstacleCollision(Eigen::MatrixXd& new_node, Eigen::MatrixXd& nearest_node, Eigen::MatrixXd& obs_position){
    Eigen::MatrixXd dist_temp = new_node - nearest_node;
    double dist = dist_temp.norm();
    Eigen::MatrixXd vector;
    if(dist == 0){
        vector = dist_temp * dist;
    }else{
        vector = dist_temp/dist;
    }
    double step = dist / Manipulator::step_div;

    Eigen::MatrixXd state_angle;
    Eigen::MatrixXd joint_position;

    Eigen::MatrixXd obs_dist;
    for(int i = 0; i < Manipulator::obstacle_num; ++i){
        for(int j = 1; j <= Manipulator::step_div; ++j){
            state_angle = nearest_node + j*step*vector;
            joint_position = Manipulator::fkine(state_angle);
            obs_dist = (joint_position - obs_position.col(i).replicate(1, 4)).colwise().norm();
            float dist = 65536;
            for(int k = 0; k < 3; ++k){
                if(obs_dist(0, k) + obs_dist(0, k+1) == Manipulator::link_length[k]){
                    return 0;
                }else if(pow(obs_dist(0, k+1), 2) >= pow(obs_dist(0, k), 2) + pow(Manipulator::link_length[k], 2)){
                    if(obs_dist(0, k) < dist){
                        dist = obs_dist(0, k);
                    }
                }else if(pow(obs_dist(0, k), 2) >= pow(obs_dist(0, k+1), 2) + pow(Manipulator::link_length[k], 2)){
                    if(obs_dist(0, k+1) < dist){
                        dist = obs_dist(0, k+1);
                    }
                }else{
                    float p = (obs_dist(0, k) + obs_dist(0, k+1) + Manipulator::link_length[k])/2;
                    float S = sqrt(p*(p - obs_dist(0, k))*(p - obs_dist(0, k+1))*(p - Manipulator::link_length[k]));
                    float h = 2*S/Manipulator::link_length[k];
                    if(h < dist){
                        dist = h;
                    }
                }
            }
            dist = dist - Manipulator::arm_radius - Manipulator::obs_radius[i];
            if(dist < 0){
                return 0;
            }
        }
    }
    return 1;
}

void Manipulator::insertNode(Eigen::MatrixXd& new_node, int& nearest_node_ind, int& new_node_ind){
    new_node_ind = Manipulator::node_added;
    Manipulator::tree.col(Manipulator::node_added) = new_node;
    Manipulator::parent(0, Manipulator::node_added) = nearest_node_ind;
    Manipulator::children(0, nearest_node_ind) += 1;
    Manipulator::sum_cost(0, Manipulator::node_added) = Manipulator::sum_cost(0, nearest_node_ind)
    + (new_node - Manipulator::tree.col(nearest_node_ind)).norm();
    Manipulator::node_added = Manipulator::node_added + 1;
}

void Manipulator::findPath(int nearest_node_index){
    while(!Manipulator::back_trace.empty()){
        Manipulator::back_trace.pop();
    }
    int current_index = nearest_node_index;
    int path_iter = 0;
    while(current_index != Manipulator::root_node ){
        Manipulator::back_trace.push(current_index);
        path_iter = path_iter + 1;
        current_index = Manipulator::parent(0, current_index);
    }
    Manipulator::back_trace.push(current_index);
}

void Manipulator::chooseParent(Eigen::MatrixXd& new_node, std::vector<std::vector<int> >& neighbor_nodes,
                               int& nearest_node_ind, Eigen::MatrixXd& obs_position, int& new_node_ind){
    double sum_cost;
    int parent = nearest_node_ind;
    double min_cost = Manipulator::sumCost(nearest_node_ind, sum_cost) + (Manipulator::tree.col(nearest_node_ind) - new_node).norm();
    double temp_cost;
    for(int i = 0; i < neighbor_nodes[0].size(); ++i){
//        std::cout << neighbor_nodes[0].size() << std::endl;
//        std::cout << "neighbor_nodes[0][i]: "  << neighbor_nodes[0][i] <<std::endl;
        if(Manipulator::obstacleCollision(new_node, neighbor_nodes[0][i], obs_position)){
            sum_cost = 0;
            temp_cost = Manipulator::sumCost(neighbor_nodes[0][i], sum_cost) + (Manipulator::tree.col(neighbor_nodes[0][i]) - new_node).norm();
            if(temp_cost < min_cost){
                min_cost = temp_cost;
                parent = neighbor_nodes[0][i];
            }
//            std::cout << "************" << std::endl;
//            std::cout << "cost: " <<min_cost << "   parent: " << parent <<std::endl;
        }
    }
    //insertNode
    new_node_ind = Manipulator::node_added;
    Manipulator::tree.col(new_node_ind) = new_node;
    Manipulator::parent(0, new_node_ind) = parent;
    Manipulator::sum_cost(0, new_node_ind) = min_cost;
    Manipulator::children(0, parent) += 1;
    Manipulator::node_added = Manipulator::node_added + 1;
}

void Manipulator::rewire(Eigen::MatrixXd& new_node, std::vector<std::vector<int> >& neighbor_nodes, Eigen::MatrixXd& obs_position){
    int new_node_ind = Manipulator::node_added - 1;
    float temp_cost;
    for(int i = 0; i < neighbor_nodes[0].size(); ++i){
        if(Manipulator::obstacleCollision(new_node, neighbor_nodes[0][i], obs_position)){
            temp_cost = Manipulator::sum_cost(0, new_node_ind) + (Manipulator::tree.col(neighbor_nodes[0][i]) - new_node).norm();
            if(temp_cost < Manipulator::sum_cost(0, neighbor_nodes[0][i])){
                Manipulator::sum_cost(0, neighbor_nodes[0][i]) = temp_cost;
                Manipulator::children(0, new_node_ind) += 1;
                Manipulator::children(0, Manipulator::parent(0, neighbor_nodes[0][i])) -= 1;
                Manipulator::parent(0, neighbor_nodes[0][i]) = new_node_ind;
            }
        }
    }
}

double Manipulator::sumCost(int& nearest_node_ind, double sum_cost){
    int current_index = nearest_node_ind;
    while(current_index != Manipulator::root_node){
//        std::cout << "*****current_index*****" << std::endl;
//        std::cout << current_index << std::endl;
        sum_cost += (Manipulator::tree.col(current_index) - Manipulator::tree.col(Manipulator::parent(0, current_index))).norm();
        current_index = Manipulator::parent(0, current_index);
    }
    Manipulator::sum_cost(0, nearest_node_ind) = sum_cost;
    return sum_cost;
}


