//
// Created by 尤皓 on 2020/3/12.
//

//主函数
#include <pcl/io/pcd_io.h>
#include "fitting.h"
#include "DBSCAN.h"
using namespace std;
using namespace pcl;
using namespace Eigen;

typedef PointXYZ PointT;

int main() {
    PointCloud<PointT>::Ptr cloud(new PointCloud<PointT>);
    string ss("/Users/youhao/PointCloud/tj-top-revised.pcd");
    io::loadPCDFile(ss, *cloud);

    vector<double>X, Y, Z;
//    for (int i_point = 0; i_point < cloud->size(); i_point++)
//    {
//        X.push_back(cloud->points[i_point].x);
//        Y.push_back(cloud->points[i_point].y);
//        Z.push_back(cloud->points[i_point].z);
//    }
    vector<double>x_mean, y_mean, z_mean;
    PointCloud<PointT>::Ptr point_mean(new PointCloud<PointT>);




    //DBSCAN
    vector<vector <double> > ivec;
    vector<vector <double> > ivec_real;
    for (int i_point = 0; i_point < cloud->size(); i_point++)
    {
        ivec.push_back({ cloud->points[i_point].x,cloud->points[i_point].y});
        ivec_real.push_back({ 0 ,cloud->points[i_point].y});
    }
    vector<vector <double> > cluster = myDBSCAN(3, 1000, ivec_real);
    pcl::PointCloud<pcl::PointXYZ> newcloud1;
    pcl::PointCloud<pcl::PointXYZ> newcloud2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr newcloud1_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr newcloud2_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ pclPnt;
    cout<<endl<<"第1条铁轨:";
    for(int j=0;j<cluster[0].size();j++) {
        cout<<pclPnt.x<<" "<<pclPnt.y<<"  "<<pclPnt.z;
        pclPnt.x = ivec [cluster[0][j]] [0];
        pclPnt.y = ivec [cluster[0][j]] [1];
        pclPnt.z = 0;
        newcloud1.push_back(pclPnt);

    }
    cout<<endl<<"第2条铁轨:";
    for(int j=0;j<cluster[1].size();j++) {
        cout<<pclPnt.x<<" "<<pclPnt.y<<"  "<<pclPnt.z;
        pclPnt.x = ivec [cluster[1][j]] [0];
        pclPnt.y = ivec [cluster[1][j]] [1];
        pclPnt.z = 0;
        newcloud2.push_back(pclPnt);
    }





    for (int i_point = 0; i_point < newcloud1.size(); i_point++)
    {
        X.push_back(newcloud2.points[i_point].x);
        Y.push_back(newcloud2.points[i_point].y);
        Z.push_back(newcloud2.points[i_point].z);
    }
    newcloud1_ptr=newcloud1.makeShared();
    newcloud2_ptr=newcloud2.makeShared();

    double a, b, c,k_line, b_line;
    fitting fit_;
    fit_.setinputcloud(newcloud2_ptr);//点云输入






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

