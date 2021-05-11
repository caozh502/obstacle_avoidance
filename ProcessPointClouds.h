//
// Created by caleb on 03.05.21.
//

#ifndef PCFILTER_PROCESSPOINTCLOUDS_H
#define PCFILTER_PROCESSPOINTCLOUDS_H

#include <iostream>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include "Box.h"

namespace obstacle_detection {
    // shorthand for point cloud pointer
    template<typename PointT>
    using PtCdtr = typename pcl::PointCloud<PointT>::Ptr;

    template<typename PointT>
    class ProcessPointClouds {
    public:

        //constructor
        ProcessPointClouds();

        //de-constructor
        ~ProcessPointClouds();

        PtCdtr<PointT> FilterCloud(PtCdtr<PointT> cloud, float filterRes);

        std::pair<PtCdtr<PointT>, PtCdtr<PointT>>
        RansacSegmentPlane(PtCdtr<PointT> cloud, int maxIterations, float distanceTol);

        std::vector<PtCdtr<PointT>>EuclidCluster(PtCdtr<PointT> cloud, float clusterTolerance, int minSize,int maxSize);

        PtCdtr<PointT> loadPcd(std::string file);

        std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

        void renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id);

        void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const typename pcl::PointCloud<PointT>::Ptr& cloud,std::string name);

        Box BoundingBox(PtCdtr<PointT> cluster);
    };
}
#endif //PCFILTER_PROCESSPOINTCLOUDS_H
