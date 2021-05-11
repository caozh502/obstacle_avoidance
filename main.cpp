#include "ProcessPointClouds.h"
#include "ProcessPointClouds.cpp"
#include "CollisionDetection.h"
//using namespace std;
//using namespace pcl;


using namespace obstacle_detection;


int color_bar[][3] =
        {
                { 255,0,0},
                { 0,255,0 },
                { 0,0,255 },
                { 0,255,255 },
                { 255,255,0 },
                { 255,255,255 },
                { 255,0,255 }
        };

void obstacleDetection(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZRGB> *pointProcessorI,
                                      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &inputCloud) {
    //parameter
    //Filter
    float filterRes = 0.1;
    //Segmentation
    float distanceThreshold = 0.15;
    int maxIterations = 40;
    //Clustering
    float clusterTolerance = 0.5;
    int minsize = 100;
    int maxsize = 50000;
    //create uav box
    Eigen::Vector3f uav_point1(-1,0.5,10);
//    Eigen::Vector3f uav_point2(-15,1,2);
    float length_1=10;
    float width_1=-2*uav_point1(0);

//    float length_2=2*uav_point2(2);
//    float width_2=-2*uav_point2(0);

    float height=2*uav_point1(1);

    Box uav_1 = CreateBox(uav_point1,width_1,length_1,height);
//    Box uav_2 = CreateBox(uav_point2,width_2,length_2,height);



    // First:Filter cloud to reduce amount of points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, filterRes);
    // Second: Segment the filtered cloud into obstacles and road
    std::pair<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segmentCloud = pointProcessorI->RansacSegmentPlane(
            filteredCloud, maxIterations, distanceThreshold);
    pointProcessorI->renderPointCloud(viewer, segmentCloud.first, "planeCloud");
    // Third: Cluster different obstacle cloud
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloudClusters = pointProcessorI->EuclidCluster(segmentCloud.second,
                                                                                                           clusterTolerance,
                                                                                                           minsize, maxsize);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cluster : cloudClusters) {

        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_in_color_h(cluster,
                                                                                            color_bar[clusterId][0],
                                                                                            color_bar[clusterId][1],
                                                                                            color_bar[clusterId][2]);//赋予显示点云的颜色
//        pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGB> cloud_in_color_h(cluster);
        viewer->addPointCloud<pcl::PointXYZRGB>(cluster, cloud_in_color_h, "cluster" + std::to_string(clusterId));
        clusterId++;
        // Fourth: Find bounding boxes for each obstacle cluster
        Box box = pointProcessorI->BoundingBox(cluster);
        pointProcessorI->renderBox(viewer, box, clusterId);

        // 5th: Collision Detection
        UAVBoxVisualization(viewer, uav_1, colors[1], clusterId);
//        CollisionVisualization(viewer, uav_2, color_bar[1], clusterId * 2 + 2);
        if (isCollision(box, uav_1)) {
            std::cout << "find Collision on the front" << std::endl;
            viewer->removeShape("uav" + std::to_string(clusterId));
            CollisionVisualization(viewer, uav_1, colors[0], clusterId);
        }
        clusterId++;
//        if (isCollision(box, uav_2)) {
//            std::cout << "find Collision on the side" << std::endl;
//            viewer->removeShape("uav" + std::to_string(clusterId * 2 + 2));
//            CollisionVisualization(viewer, uav_2, color_bar[0], clusterId * 2 + 2);
//        }
    }
}

//Box CreateBox(Eigen::Vector3d uav_point, float width, float length, float height){
//    Box uav;
//    uav.x_min=uav_point(0);
//    uav.x_max=uav_point(0)+width;
//    uav.y_min=uav_point(1);
//    uav.y_max=uav_point(1)+height;
//    uav.z_min=uav_point(2)+length;
//    uav.z_max=uav_point(2);
//    return uav;
//}


int main(int argc, char** argv)
{

    //initialization
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));


    //  Stream ObstacleDetection function
    auto *pointProcessorI = new ProcessPointClouds<pcl::PointXYZRGB>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("/home/caleb/pcd");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inputCloudI;

    viewer->setBackgroundColor(0.6, 0.6, 0.6);
    while (!viewer->wasStopped())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        obstacleDetection(viewer, pointProcessorI, inputCloudI);
        streamIterator++;
        if (streamIterator == stream.end()) {
            streamIterator = stream.begin();
        }
        viewer->spinOnce(75);
    }

    return(0);
}