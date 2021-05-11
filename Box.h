//
// Created by caleb on 10.05.21.
//

#ifndef PCFILTER_BOX_H
#define PCFILTER_BOX_H
#include <Eigen/Geometry>

//struct BoxQ
//{
//    Eigen::Vector3f bboxTransform;
//    Eigen::Quaternionf bboxQuaternion;
//    float cube_length;
//    float cube_width;
//    float cube_height;
//};
struct Box
{
    float x_min;
    float y_min;
    float z_min;
    float x_max;
    float y_max;
    float z_max;
};

#endif //PCFILTER_BOX_H
