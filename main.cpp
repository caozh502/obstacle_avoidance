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

using namespace std;
using namespace pcl;

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


void groundRemove(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered)
{
    auto startTime = std::chrono::steady_clock::now();
    if (cloud->size() > 0)
    {
        //创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // 创建分割对象
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // 可选择配置，设置模型系数需要优化
        seg.setOptimizeCoefficients(true);
        // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // you can modify the parameter below
        seg.setMaxIterations(100);
        seg.setDistanceThreshold(0.15);
        seg.setInputCloud(cloud);
        //引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            cout<<"error! Could not found any inliers!"<<endl;
        }
        // extract ground
        pcl::ExtractIndices<pcl::PointXYZRGB> extractor;
        extractor.setInputCloud(cloud);
        extractor.setIndices(inliers);
        extractor.setNegative(true);
        extractor.filter(*cloud_filtered);
        // vise-versa, remove the ground not just extract the ground
        // just setNegative to be true
        cout << "Ground removal done."<<endl;
    }
    else
    {
        cout<<"no data!"<<endl;
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ground removal took " << elapsedTime.count() << " ms" << std::endl;
}

void FilterCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered){
    auto startTime = std::chrono::steady_clock::now();

    if (cloud->size() > 0){
        //ROI filter
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fz(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fy(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_fx(new pcl::PointCloud<pcl::PointXYZRGB>);
        //z轴直通滤波
        pcl::PassThrough<pcl::PointXYZRGB> pass_z;//设置滤波器对象
        //参数设置
        pass_z.setInputCloud(cloud);
        pass_z.setFilterFieldName("z");
        //z轴区间设置
        pass_z.setFilterLimits(0,20);
        //设置为保留还是去除(true为去除)
        //pass_z.setFilterLimitsNegative(true);
        pass_z.filter(*cloud_fz);

        //y轴直通滤波
        pcl::PassThrough<pcl::PointXYZRGB> pass_y;//设置滤波器对象
        pass_y.setInputCloud(cloud_fz);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-3, 3);
        //pass_y.setFilterLimitsNegative(false);
        pass_y.filter(*cloud_fy);
        //x轴直通滤波
        pcl::PassThrough<pcl::PointXYZRGB> pass_x;//设置滤波器对象
        pass_x.setInputCloud(cloud_fy);
        pass_x.setFilterFieldName("x");
        pass_x.setFilterLimits(-15, 15);
        //pass_x.setFilterLimitsNegative(true);
        pass_x.filter(*cloud_fx);

        pcl::VoxelGrid<pcl::PointXYZRGB> vox;
        vox.setInputCloud(cloud_fx);
        vox.setLeafSize(0.1f,0.1f,0.1f);//体素网格大小
        vox.filter(*cloud_filtered);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    cout << "before_filter :"<<cloud->points.size()<<endl;
    cout << "after filter :" << cloud_filtered->points.size()<<endl;
    cout << "Voxel grid took " << elapsedTime.count() << " ms" << endl;
}

void EuclidCluster(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::vector<pcl::PointCloud<PointXYZRGB>::Ptr> &clusters){

    auto startTime = std::chrono::steady_clock::now();
    // KdTree的搜索方式
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    // 欧几里德聚类提取
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.2); // 设置空间聚类误差2cm
    ec.setMinClusterSize(100);  // 设置有效聚类包含的最小的个数
    ec.setMaxClusterSize(25000);  // 设置有效聚类包含的最大的个数
    ec.setSearchMethod(tree);  // 设置搜索方法
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);  // 获取切割之后的聚类索引保存到cluster_indices

    // For each cluster indices
    for (pcl::PointIndices getIndices: cluster_indices){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        // For each point indices in each cluster
        for (int index:getIndices.indices){
            cloud_cluster->points.push_back(cloud->points[index]);} //*
        cloud_cluster->width = cloud_cluster->points.size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Clustering took " << elapsedTime.count() << " ms and found " << clusters.size()
              << " clusters" << std::endl;

}

void renderBox(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster, int id){
    // Find bounding box for one of the clusters
    pcl::PointXYZRGB minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    std::string cube = "box"+std::to_string(id);
    viewer->addCube(minPoint.x, maxPoint.x, minPoint.y, maxPoint.y, minPoint.z,maxPoint.z, 1, 0, 0, cube);
    viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
}

void Visualisation(boost::shared_ptr<pcl::visualization::PCLVisualizer>& viewer, const std::vector<pcl::PointCloud<PointXYZRGB>::Ptr> cloud_cluster){
    viewer->setBackgroundColor(0, 0, 0);
    int j=0;
    for (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster : cloud_cluster) {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_in_color_h(cluster,
                                                                                            color_bar[j][0],
                                                                                            color_bar[j][1],
                                                                                            color_bar[j][2]);//赋予显示点云的颜色
        viewer->addPointCloud<pcl::PointXYZRGB>(cluster, cloud_in_color_h, "cluster" + to_string(j));
        j++;
        renderBox(viewer, cluster, j);
    }
    viewer->addCoordinateSystem();
}

int main(int argc, char** argv)
{

    //initialization
    pcl::PCDReader reader;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    reader.read("../tree.pcd",*cloud);

    PointCloud<pcl::PointXYZRGB>::Ptr cloud(new PointCloud<pcl::PointXYZRGB>);
    PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new PointCloud<pcl::PointXYZRGB>);
    PointCloud<pcl::PointXYZRGB>::Ptr cloud_noGround(new PointCloud<pcl::PointXYZRGB>);
    std::vector<pcl::PointCloud<PointXYZRGB>::Ptr> cloud_cluster;

    //Voxel grid filter
    FilterCloud(cloud, cloud_f);


    //ground removal
    groundRemove(cloud_f, cloud_noGround);

    //Clustering
    EuclidCluster(cloud_noGround,cloud_cluster);

    //Visualisation
    Visualisation(viewer, cloud_cluster);

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }

    return(0);
}