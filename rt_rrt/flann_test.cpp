////
////  flann_test.cpp
////  rt_rrt
////
////  Created by 谢龙 on 2017/12/14.
////  Copyright © 2017年 long. All rights reserved.
////
//
//#include <stdio.h>
//#include <cmath>
//#include <eigen3/Eigen/Geometry>
//#include <flann/flann.hpp>
//
//int main(int argc, const char * argv[]){
//    Eigen::MatrixXf joint;
//    Eigen::MatrixXf joint_temp(5,1);
//    joint_temp << 1,4,2,6,2;
//    joint = joint_temp;
//    flann::Matrix<float> data_set(joint.data(), 5, 1);
//    flann::Index<flann::L2<float>> kd_tree(data_set, flann::KDTreeIndexParams(4));
////    flann::Matrix<float> point(joint.data(), 1, 1);
//    Eigen::Matrix<float, 1, 1, Eigen::RowMajor> joint1;
//    joint1 << 0;
//    flann::Matrix<float> point1(joint1.data(), 1, 1);
//    kd_tree.addPoints(point1);
//    kd_tree.buildIndex();
//
//
//    float query[1] = {0.4};
//    flann::Matrix<float> query_node(query, 1, 1);
//    std::vector<std::vector<int> > index;
//    std::vector<std::vector<float> > dist;
////    kd_tree.knnSearch(query_node, index, dist, 2, flann::SearchParams(128));
////    for(int i = 0; i < 2; ++i){
////        kd_tree.knnSearch(query_node, index, dist, 2, flann::SearchParams(128));
////        kd_tree.addPoints(point1);
////        std::cout << "index size: " << index.size() << std::endl;
////        for(int i = 0; i < index.size(); ++i){
////            std::cout << "index: " << index[i][0] << std::endl;
////            std::cout << "dist: " << sqrt(dist[i][0]) << std::endl;
////        }
////    }
//    kd_tree.radiusSearch(query_node, index, dist, 31.35, flann::SearchParams(128));
//    for(int i = 0; i < index[0].size(); ++i){
//        std::cout << "*************" << std::endl;
//        std::cout << "index size: " << index[0].size() << std::endl;
//        std::cout << "index: " << index[0][i] << std::endl;
//        std::cout << "dist: " << sqrt(dist[0][i]) << std::endl;
//    }
//    std::cout << "End" << std::endl;
//    return 0;
//}
//
//
//
//
