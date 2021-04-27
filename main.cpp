#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>

using namespace std;
using namespace pcl;

void detectObjectsOnCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered)
{
    if (cloud->size() > 0)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // you can modify the parameter below
        seg.setMaxIterations(10000);
        seg.setDistanceThreshold(0.15);
        seg.setInputCloud(cloud);
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
        cout << "filter done."<<endl;
    }
    else
    {
        cout<<"no data!"<<endl;
    }
}
int main(int argc, char** argv)
{

    //读取点云
    pcl::PCDReader reader;
    PointCloud<pcl::PointXYZRGB>::Ptr cloud(new PointCloud<pcl::PointXYZRGB>);
    PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new PointCloud<pcl::PointXYZRGB>);
    reader.read("../tree.pcd",*cloud);
    cout << "before_filter :"<<cloud->points.size()<<endl;

    //体素滤波
    pcl::VoxelGrid<pcl::PointXYZRGB> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(0.1f,0.1f,0.1f);//体素网格大小
    vox.filter(*cloud_f);
    cout << "after filter :" << cloud_f->points.size()<<endl;

    detectObjectsOnCloud(cloud, cloud_f);
    //点云可视化
    pcl::visualization::PCLVisualizer viewer("cloud_viewer");
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud(cloud_f);
    viewer.addCoordinateSystem();

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return(0);
}