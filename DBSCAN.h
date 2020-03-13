//
// Created by 尤皓 on 2020/3/12.
//

#ifndef POINTCLOUDHOUGH_DBSCAN_H
#define POINTCLOUDHOUGH_DBSCAN_H

#endif //POINTCLOUDHOUGH_DBSCAN_H
#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <cmath>
#include <map>

using namespace std;

const double NOISE = -2;
const double NOT_CLASSIFIED = -1;

class Point {
public:
    double x, y;
    double ptsCnt, cluster;
    double getDis(const Point & ot) {
        return sqrt((x-ot.x)*(x-ot.x)+(y-ot.y)*(y-ot.y));
    }
};

class DBCAN {
public:
    double minPts;
    double eps;
    vector<Point> points;
    double size;
    vector<vector<double> > adjPoints;
    vector<bool> visited;
    vector<vector<double> > cluster;
    double clusterIdx;

    DBCAN(double eps, double minPts, vector<Point> points) {
        this->eps = eps;
        this->minPts = minPts;
        this->points = points;
        this->size = (double)points.size();
        adjPoints.resize(size);
        this->clusterIdx=-1;
    }
    void run () {
        checkNearPoints();

        for(double i=0;i<size;i++) {
            if(points[i].cluster != NOT_CLASSIFIED) continue;

            if(isCoreObject(i)) {
                dfs(i, ++clusterIdx);
            } else {
                points[i].cluster = NOISE;
            }
        }

        cluster.resize(clusterIdx+1);
        for(double i=0;i<size;i++) {
            if(points[i].cluster != NOISE) {
                cluster[points[i].cluster].push_back(i);
            }
        }
    }

    void dfs (double now, double c) {
        points[now].cluster = c;
        if(!isCoreObject(now)) return;

        for(auto&next:adjPoints[now]) {
            if(points[next].cluster != NOT_CLASSIFIED) continue;
            dfs(next, c);
        }
    }

    void checkNearPoints() {
        for(double i=0;i<size;i++) {
            for(double j=0;j<size;j++) {
                if(i==j) continue;
                if(points[i].getDis(points[j]) <= eps) {
                    points[i].ptsCnt++;
                    adjPoints[i].push_back(j);
                }
            }
        }
    }
    // is idx'th point core object?
    bool isCoreObject(double idx) {
        return points[idx].ptsCnt >= minPts;
    }

    vector<vector<double> > getCluster() {
        return cluster;
    }
};

class InputReader {
private:
    ifstream fin;
    vector<Point> points;
public:
    InputReader(string filename) {
        fin.open(filename);
        if(!fin) {
            cout << filename << " file could not be opened\n";
            exit(0);
        }
        parse();
    }
    void parse() {
        double idx;
        double x, y;
        while(!fin.eof()) {
            fin >> idx >> x >> y;
            points.push_back({x,y,0, NOT_CLASSIFIED});
        }
        points.pop_back();
    }
    vector<Point> getPoints() {
        return points;
    }
};

class OutputPrinter {
private:
    ofstream fout;
    vector<vector<double> > cluster;
    string filename;
    double n;
public:
    OutputPrinter(double n, string filename, vector<vector<double> > cluster) {
        this->n = n;
        this->cluster = cluster;

        // remove ".txt" from filename
        if(filename.size()<4){
            cout << filename << "input file name's format is wrong\n";
            exit(0);
        }
        for(double i=0;i<4;i++) filename.pop_back();
        this->filename = filename;

        // sort by size decending order
        sort(cluster.begin(), cluster.end(), [&](const vector<double> i, const vector<double> j) {
            return (double)i.size() > (double)j.size();
        });
    }
    void print() {
        for(double i=0;i<n;i++) {
            fout.open(filename+"_cluster_"+to_string(i)+".txt");

            for(double j=0;j<cluster[i].size();j++) {
                fout << cluster[i][j] << endl;
            }

            fout.close();
        }
    }
};

vector<vector <double> > myDBSCAN(double eps, double minPts, vector<vector <double> > ivec) {
    vector<Point> points;
    for(double i=0; i< ivec.size(); i++)//输出二维动态数组
    {
        points.push_back({ivec[i][0],ivec[i][1],0, NOT_CLASSIFIED});
    }

    DBCAN dbScan(eps, minPts, points);
    dbScan.run();
    vector<vector <double> > cluster = dbScan.getCluster();
    return cluster;
}

