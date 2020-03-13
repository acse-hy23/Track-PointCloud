#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

pcl::PointXYZ change1(pcl::PointXYZ pclPnt){
    double temp;
    temp = pclPnt.x;
    pclPnt.x = pclPnt.y;
    pclPnt.y = temp;
    pclPnt.x*=5000;
    pclPnt.y*=5000;
    pclPnt.z*=5000;
    pclPnt.x-=400;
    return pclPnt;
}

pcl::PointXYZ change2(pcl::PointXYZ pclPnt){
    double temp;
    temp = pclPnt.z;
    pclPnt.z = pclPnt.y;
    pclPnt.y = temp;
    return pclPnt;
}


int main(int argc,char** argv)    // pcl 函数名的方式
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if(pcl::io::loadPCDFile<pcl::PointXYZ>("/Users/youhao/PointCloud/cha-revised.pcd",*cloud)==-1)//*打开点云文件
    {
        PCL_ERROR("Couldn't read file test_pcd.pcd\n");
        return(-1);
    }
    std::cout<<"Loaded "
             <<cloud->width*cloud->height
             <<" data points from test_pcd.pcd with the following fields: "
             <<std::endl;

    for(size_t i=0;i<cloud->points.size();++i){
        cloud->points[i] = change2(cloud->points[i]);
        std::cout<<"   "<<cloud->points[i].x<<"   "<<cloud->points[i].y<<"   "<<cloud->points[i].z<<std::endl;
    }

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("/Users/youhao/PointCloud/c", *cloud, false);
}

