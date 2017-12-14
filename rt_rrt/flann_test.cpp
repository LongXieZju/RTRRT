////
////  flann_test.cpp
////  rt_rrt
////
////  Created by 谢龙 on 2017/12/14.
////  Copyright © 2017年 long. All rights reserved.
////
//
//#include <stdio.h>
//#include <eigen3/Eigen/Geometry>
//#include <flann/flann.hpp>
//
//int main(int argc, const char * argv[]){
//    Eigen::MatrixXf joint;
//    Eigen::MatrixXf joint_temp(7,1);
//    joint_temp << 1,2,3,4,5,6,7;
//    joint = joint_temp;
//    flann::Matrix<float> data_set(joint.data(), 1, 7);
//    flann::Index<flann::L2<float>> kd_tree(data_set, flann::KDTreeIndexParams(4));
//    flann::Matrix<float> point(joint.data(), 1, 7);
//    Eigen::Matrix<float, 1, 7, Eigen::RowMajor> joint1;
//    joint1 << 0,0,0,0,0,0,0;
//    flann::Matrix<float> point1(joint1.data(), 1, 7);
//    kd_tree.addPoints(point1);
//    kd_tree.buildIndex();
//
//
//    float query[7] = {0,0,0,0,1,1,1};
//    flann::Matrix<float> query_node(query, 1, 7);
//    std::vector<std::vector<int> > index;
//    std::vector<std::vector<float> > dist;
//    kd_tree.knnSearch(query_node, index, dist, 2, flann::SearchParams(128));
//    for(int i = 0; i < 10; ++i){
//
//        kd_tree.knnSearch(query_node, index, dist, 2, flann::SearchParams(128));
//        kd_tree.addPoints(point1);
//        std::cout << "index size: " << index.size() << std::endl;
//        for(int i = 0; i < index.size(); ++i){
//            std::cout << "index: " << index[i][0] << std::endl;
//            std::cout << "dist: " << dist[i][0] << std::endl;
//        }
//    }
//    std::cout << "End" << std::endl;
//    return 0;
//}
//
//
//
