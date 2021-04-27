#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace std;
using namespace pcl;

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