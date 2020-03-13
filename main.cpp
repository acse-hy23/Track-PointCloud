#include <iostream>
#include<cmath>
#include "HOUGH_LINE.h"
#include "DBSCAN.h"
int main()
{
    PointCloud<PointT>::Ptr cloud(new PointCloud<PointT>);
    pcl::io::loadPCDFile<PointT>("/Users/youhao/PointCloud/cha.pcd", *cloud);
    PointCloud<PointT>::Ptr VOXEL;
    HOUGH_LINE hough;
    hough.setinputpoint(cloud);
    //hough.VoxelGrid_(1.0, VOXEL);
    hough.draw_hough_spacing();
    vector<float>K_,B_;
    //hough.HOUGH_line(600, 2, 50, K_, B_);//按阈值自动检测
    int line_num = 10;
    hough.HOUGH_line(5000, 0.01, 10,K_, B_, line_num);//指定只选择极大值最大的3条直线
    hough.draw_hough_line();

    vector<vector <double> > ivec(line_num ,vector<double>(2,0));    //m*n的二维vector，所有元素为0
    for(int i=0; i< ivec.size(); i++)//输出二维动态数组
    {
            ivec[i][0] = K_[i];
            ivec[i][1] = B_[i];
            //cout<<ivec[i][0]<<" "<<ivec[i][1]<<endl;
    }

    int tracknum = 2;//铁轨数量，但是DBSCAN算法无法指定该参数。

    vector<vector<vector<double>>> track(2);//创建2个vector<vector<int> >类型的数组
    vector<vector <double> > cluster = myDBSCAN(2, 1, ivec);
    for(int i=0;i<tracknum;i++) {
        cout<<endl<<"第"<<i<<"条铁轨:";
        for(int j=0;j<cluster[i].size();j++) {
            track[i].push_back(vector<double>{ivec [cluster[i][j]] [0] , ivec [cluster[i][j]] [1] });
            cout<< cluster[i][j] << "  ";
        }
    }
    double ksum[2],bsum[2];
    cout<<endl<<endl<<"-------------------"<<endl;
    for (int i = 0; i<2; i++){
        for (const auto& inner : track[i]) {
            ksum[i]+=inner[0];
            bsum[i]+=inner[i];
            cout<<inner[0]<<"  "<< inner[1]<<endl;
        }
        cout<<"-------------------"<<endl;
    }
    double k1 = ksum[0]/track[0].size();
    double k2 = ksum[1]/track[1].size();
    double b1 = bsum[0]/track[0].size();
    double b2 = bsum[1]/track[1].size();
    double k_ave = (k1+k2)/2;
    double distance = abs(b1-b2)/pow(k_ave*k_ave+1,0.5);
    cout<<endl<<"第一条铁轨"<<k1<<" "<<b1<<endl<<"第二条铁轨"<<k2<<" "<<b2<<endl<<"k平均"<<k_ave<<endl<<"轨距"<<distance;
    return 0;
}


