//
// Created by caleb on 03.05.21.
//

#include "ProcessPointClouds.h"

using namespace obstacle_detection;

//constructor
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template<typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::FilterCloud(PtCdtr<PointT> cloud, float filterRes){
    auto startTime = std::chrono::steady_clock::now();

    PtCdtr<PointT> cloud_filtered(new pcl::PointCloud<PointT>);
    PtCdtr<PointT> cloud_fz(new pcl::PointCloud<PointT>);
    PtCdtr<PointT> cloud_fy(new pcl::PointCloud<PointT>);
    PtCdtr<PointT> cloud_fx(new pcl::PointCloud<PointT>);

    //ROI filter
    //z轴直通滤波(深度）
    pcl::PassThrough<PointT> pass_z;//设置滤波器对象
    //参数设置
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    //z轴区间设置
    pass_z.setFilterLimits(0,15);
    //设置为保留还是去除(true为去除)
    //pass_z.setFilterLimitsNegative(true);
    pass_z.filter(*cloud_fz);

    //y轴（高度）
    pcl::PassThrough<PointT> pass_y;//设置滤波器对象
    pass_y.setInputCloud(cloud_fz);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-3, 3);
    //pass_y.setFilterLimitsNegative(false);
    pass_y.filter(*cloud_fy);
    //x轴（宽度）
    pcl::PassThrough<PointT> pass_x;//设置滤波器对象
    pass_x.setInputCloud(cloud_fy);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-5, 5);
    //pass_x.setFilterLimitsNegative(true);
    pass_x.filter(*cloud_fx);
    //VoxelGrid filter
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud(cloud_fx);
    vox.setLeafSize(filterRes,filterRes,filterRes);//体素网格大小
    vox.filter(*cloud_filtered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//    cout << "before_filter :"<<cloud->points.size()<<endl;
//    cout << "after filter :" << cloud_filtered->points.size()<<endl;
    cout << "filtering took " << elapsedTime.count() << " ms" << endl;

    return cloud_filtered;
}

template<typename PointT>
std::pair<PtCdtr<PointT>, PtCdtr<PointT>>
ProcessPointClouds<PointT>::RansacSegmentPlane(PtCdtr<PointT> cloud, int maxIterations, float distanceTol){
    auto startTime = std::chrono::steady_clock::now();
    PtCdtr<PointT> out_plane(new pcl::PointCloud<PointT>());
    PtCdtr<PointT> in_plane(new pcl::PointCloud<PointT>());


        //创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // 创建分割对象
        pcl::SACSegmentation<PointT> seg;
        // 可选择配置，设置模型系数需要优化
        seg.setOptimizeCoefficients(true);
        // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // you can modify the parameter below
        seg.setMaxIterations(maxIterations);
        seg.setDistanceThreshold(distanceTol);
        seg.setInputCloud(cloud);
        //引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            cout<<"error! Could not found any inliers!"<<endl;
        }
        // extract ground
        pcl::ExtractIndices<PointT> extractor;
        extractor.setInputCloud(cloud);
        extractor.setIndices(inliers);
        extractor.setNegative(true);
        extractor.filter(*out_plane);
        extractor.setNegative(false);
        extractor.filter(*in_plane);
        // vise-versa, remove the ground not just extract the ground
        // just setNegative to be true


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ground removal took " << elapsedTime.count() << " ms" << std::endl;

    return std::pair<PtCdtr<PointT>, PtCdtr<PointT>>(in_plane, out_plane);
}

template<typename PointT>
std::vector<PtCdtr<PointT>>
ProcessPointClouds<PointT>::EuclidCluster(PtCdtr<PointT> cloud, float clusterTolerance, int minSize,int maxSize){
    auto startTime = std::chrono::steady_clock::now();
    // Search Method: KdTree
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    // Euclidean Cluster
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(clusterTolerance); // 设置空间聚类误差20cm
    ec.setMinClusterSize(minSize);  // 设置有效聚类包含的最小的个数
    ec.setMaxClusterSize(maxSize);  // 设置有效聚类包含的最大的个数
    ec.setSearchMethod(tree);  // 设置搜索方法
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);  // 获取切割之后的聚类索引保存到cluster_indices
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    // For each cluster indices
    for (pcl::PointIndices getIndices: cluster_indices){
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        // For each point indices in each cluster
        for (int index:getIndices.indices){
            cloud_cluster->points.push_back(cloud->points[index]);}
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Clustering took " << elapsedTime.count() << " ms and found " << clusters.size()
              << " clusters" << std::endl;
    return clusters;
}

template<typename PointT>
PtCdtr<PointT> ProcessPointClouds<PointT>::loadPcd(std::string file){
    PtCdtr<PointT> cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) { //* load the file
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath){

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath},
                                               boost::filesystem::directory_iterator{});

    // sort files in ascending order so playback is chronological 按升序对文件进行排序，以便按时间顺序播放
    sort(paths.begin(), paths.end());

    return paths;
}

template<typename PointT>
void ProcessPointClouds<PointT>::renderBox(pcl::visualization::PCLVisualizer::Ptr& viewer, Box box, int id){
//    PointT minPoint, maxPoint;
//    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    std::string cube = "box"+std::to_string(id);
    viewer->addCube(box.x_min, box.x_max, box.y_min, box.y_max, box.z_min, box.z_max, 1, 0, 0, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
}

template<typename PointT>
void ProcessPointClouds<PointT>::renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, const typename pcl::PointCloud<PointT>::Ptr& cloud, std::string name){
    viewer->addPointCloud<PointT> (cloud, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.5, 0, name);
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(PtCdtr<PointT> cluster) {

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}