//
// Created by caleb on 10.05.21.
//

#ifndef PCFILTER_COLLISIONDETECTION_H
#define PCFILTER_COLLISIONDETECTION_H

//#include "fcl/config.h"
//#include "fcl/octree.h"
//#include "fcl/traversal/traversal_node_octree.h"
//#include "fcl/collision.h"
//#include "fcl/broadphase/broadphase.h"
//#include "fcl/math/transform.h"

#include "Box.h"
#include <vector>
#include <Eigen/Dense>
#include <pcl/visualization/pcl_visualizer.h>
struct Color
{

    float r, g, b;

    Color(float setR, float setG, float setB)
            : r(setR), g(setG), b(setB)
    {}
};


        Box createBox(Eigen::Vector3f uav_point, float width, float length, float height);


        bool isCollision(Box box, Box uav);

        void CollisionVisualization(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, Color color,int id);

        void UAVBoxVisualization(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, Color color,int id);




#endif //PCFILTER_COLLISIONDETECTION_H
