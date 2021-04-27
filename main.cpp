#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/passthrough.h>


using namespace std;
using namespace pcl;

void detectObjectsOnCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered)
{
    auto startTime = std::chrono::steady_clock::now();
    if (cloud->size() > 0)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        // you can modify the parameter below
        seg.setMaxIterations(100);
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
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "Ground removal took " << elapsedTime.count() << " milliseconds" << std::endl;
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
    std::cout << "Voxel grid took " << elapsedTime.count() << " milliseconds" << std::endl;
}


int main(int argc, char** argv)
{

    //read point cloud data
    pcl::PCDReader reader;
    PointCloud<pcl::PointXYZRGB>::Ptr cloud(new PointCloud<pcl::PointXYZRGB>);
    PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new PointCloud<pcl::PointXYZRGB>);
    PointCloud<pcl::PointXYZRGB>::Ptr cloud_noGround(new PointCloud<pcl::PointXYZRGB>);
    reader.read("../tree.pcd",*cloud);
    cout << "before_filter :"<<cloud->points.size()<<endl;

    //Voxel grid filter
    FilterCloud(cloud, cloud_f);
    cout << "after filter :" << cloud_f->points.size()<<endl;

    //ground removal
    detectObjectsOnCloud(cloud_f, cloud_noGround);

    //pcl_viewer
    pcl::visualization::PCLVisualizer viewer("cloud_viewer");
    viewer.setBackgroundColor(255, 255, 255);
    viewer.addPointCloud(cloud_noGround);
    viewer.addCoordinateSystem();

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return(0);
}