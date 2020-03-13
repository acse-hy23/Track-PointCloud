//
// Created by 尤皓 on 2020/3/10.
//

#ifndef POINTCLOUDHOUGH_HOUGH_LINE_H
#define POINTCLOUDHOUGH_HOUGH_LINE_H

#endif //POINTCLOUDHOUGH_HOUGH_LINE_H
//HOUGH_LINE.h文件
#pragma once
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <pcl/common/common.h>
#include <pcl/visualization/pcl_plotter.h>
#include <pcl/filters/voxel_grid.h>
#include <queue>

using namespace pcl;
using namespace Eigen;
using namespace std;
typedef PointXYZ PointT;
class HOUGH_LINE
{
public:
    HOUGH_LINE();
    ~HOUGH_LINE();
    template<typename T> void vector_sort(std::vector<T> vector_input, std::vector<size_t> &idx);
    inline void setinputpoint(PointCloud<PointT>::Ptr point_);
    void VoxelGrid_(float size_, PointCloud<PointT>::Ptr &voxel_cloud);
    void draw_hough_spacing();
    void HOUGH_line(int x_setp_num, double y_resolution, int grid_point_number_threshold, vector<float>&K_, vector<float>&B_, int line_num=-1);
    void draw_hough_line();
private:
    PointCloud<PointT>::Ptr cloud;
    int point_num;
    PointT point_min;
    PointT point_max;
    vector<pair<double, double>>result_;
};

template<typename T>void
HOUGH_LINE::vector_sort(std::vector<T> vector_input, std::vector<size_t> &idx){
    idx.resize(vector_input.size());
    iota(idx.begin(), idx.end(), 0);
    sort(idx.begin(), idx.end(), [&vector_input](size_t i1, size_t i2) { return vector_input[i1] > vector_input[i2]; });
}
inline void HOUGH_LINE::setinputpoint(PointCloud<PointT>::Ptr point_){
    cloud = point_;
    point_num = cloud->size();
}

