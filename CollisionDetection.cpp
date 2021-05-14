//
// Created by caleb on 10.05.21.
//

#include "CollisionDetection.h"

Box createBox(Eigen::Vector3f uav_point, float width, float length, float height){
    Box uav;
    uav.x_min=uav_point(0);
    uav.x_max=uav_point(0)+width;
    uav.y_min=uav_point(1);
    uav.y_max=uav_point(1)+height;
    uav.z_min=uav_point(2)-length;
    uav.z_max=uav_point(2);
    return uav;
}

bool isCollision(Box box, Box uav){
    //auto startTime = std::chrono::steady_clock::now();


    if (box.x_min <= uav.x_max && box.x_max >= uav.x_min)
        if  (box.y_min <= uav.y_max && box.y_max >= uav.y_min)

            return (box.z_min <= uav.z_max && box.z_max >= uav.z_min);

//    auto endTime = std::chrono::steady_clock::now();
//    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//    std::cout << "Collision Detection took " << elapsedTime.count() << " ms" << std::endl;
    return false;

}

void CollisionVisualization(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, Color color,int id){
    std::string cube = "uav"+std::to_string(id);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, cube);
}
void UAVBoxVisualization(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, Color color,int id){
    std::string cube = "uavbox"+std::to_string(id);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, color.r, color.g, color.b, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
}
