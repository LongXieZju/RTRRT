//
//  nodeStruct.h
//  rt_rrt
//
//  Created by 谢龙 on 2017/12/14.
//  Copyright © 2017年 long. All rights reserved.
//

#ifndef nodeStruct_h
#define nodeStruct_h

#include <eigen3/Eigen/Geometry>

typedef struct NearestNode{
    int ind;
    float nearest_dist;
}NearestNode;

#endif /* nodeStruct_h */
