#include <pcl/io/pcd_io.h>
#include "fitting.h"
#include "DBSCAN.h"
using namespace std;
using namespace pcl;
using namespace Eigen;

typedef PointXYZ PointT;
pcl::PointXYZ change1(pcl::PointXYZ pclPnt){

    pclPnt.z = 0;

    return pclPnt;
}
int main() {
    PointCloud<PointT>::Ptr cloud(new PointCloud<PointT>);
    string ss("/Users/youhao/PointCloud/c.pcd");
    io::loadPCDFile(ss, *cloud);


    //把Z的坐标去掉
//    for(size_t i=0;i<cloud->points.size();++i){
//        cloud->points[i] = change1(cloud->points[i]);
//    }


    vector<double>X, Y, Z;
    for (int i_point = 0; i_point < cloud->size(); i_point++)
    {
        X.push_back(cloud->points[i_point].x);
        Y.push_back(cloud->points[i_point].y);
        Z.push_back(cloud->points[i_point].z);
    }
    vector<double>x_mean, y_mean, z_mean;
    PointCloud<PointT>::Ptr point_mean(new PointCloud<PointT>);

    double a, b, c,k_line, b_line;
    fitting fit_;
    fit_.setinputcloud(cloud);//点云输入
    fit_.line_fitting(X, Y, k_line, b_line);//直线拟合
    fit_.display_line(X, Y, b_line, k_line);//显示拟合的直线，必须先输入常量

    fit_.polynomial2D_fitting(X, Z, a, b, c);
    fit_.display_line(X, Z, c, b, a);//显示拟合的平面多项式曲线，输入顺序为 常量，一阶系数，二阶系数

    fit_.grid_mean_xyz(0.5, -1, x_mean, y_mean, z_mean, point_mean);//0.5表示x方向的步长，-1(小于0就行)表示y方向不分段，如需分段，则设置相应步长
    fit_.grid_mean_xyz_display(point_mean);//展示均值结果

    fit_.display_point(X, Y);//显示散点
    fit_.display_point(x_mean, y_mean);//显示均值散点

    fit_.polynomial3D_fitting(x_mean, y_mean, z_mean, a, b, c);//用分段质心的均值去拟合3维曲线
    //fit_.polynomial3D_fitting(X, Y, Z, a, b, c);//直接拟合
    fit_.polynomial3D_fitting_display(0.5);//三维曲线展示

    return 0;
}


