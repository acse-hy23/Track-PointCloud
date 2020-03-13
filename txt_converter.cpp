//
// Created by 尤皓 on 2020/3/10.
//
#include<iostream>
#include<fstream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
using namespace std;



int main()
{
    fstream modelRead;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PCDWriter writer;

    modelRead.open("/Users/youhao/PointCloud/test.txt",std::ios_base::in);
    pcl::PointXYZ pclPnt;
    while(!modelRead.eof())
    {
        modelRead>>pclPnt.x>>pclPnt.y>>pclPnt.z;
        cloud.push_back(pclPnt);
    }
    modelRead.close();
    writer.write("/Users/youhao/PointCloud/test.pcd",cloud);
    return 0;
}

