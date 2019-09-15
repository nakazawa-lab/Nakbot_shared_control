#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <vector>
#include <cmath>

#include "my_robo_simulation/kdtree.h"

#ifndef MY_DWA
#define MY_DWA

// kdtree.hの利用のための、点を表すクラス
class MyPoint: public std::array<double,2>
{
public:
    static const int DIM =2;

    MyPoint(){}
    MyPoint(double x, double y){
        (*this)[0] = x;
        (*this)[1] = y;
    }
};

class MyDWA
{
private:
    std::vector<int> scan_indices, traj_indices;

    std::vector<double> lin_dists, ang_dists;

    double d_thres, th_thres;

    double lin_normDist, ang_normDist;

    // LRFのkd木
    kdt::KDTree<MyPoint> LRFkdtree;

    // LRFについてのPoints
    std::vector<MyPoint> LRFpoints;

        // treeidxのツリーのidx番目の点と、queryの点の距離(を求める。
    double cal_Dist(MyPoint query, int idx);

    void cal_lin_ang_Dist(int, int,std::vector<std::vector<double>>&);

        // 同名の関数内で呼び出す、単一の軌道に対してLRFとの再接近点を求める
    void search_LRF_Traj(sensor_msgs::LaserScan& latest_scan, std::vector<std::vector<double>>& PredictTraj);


    // kd木によって、障害物を示す点群の中から、任意の点(d,theta)に対して最短距離となる点のインデックスを求める
    // scan_index, traj_indexを求める
    int kd_tree_nnSearch(const MyPoint query);

        // LRFのスキャン点から、kdtreeを構築する
    void make_kd_tree(sensor_msgs::LaserScan& scan);

public:
    MyDWA(){
    };

    MyDWA(double d_thres, double th_thres){
    };

    // 最短距離となる2つの点がわかっている状態で、lin_normDistとang_normDistを求める
    void cal_Dist();



    // 現在わかっているLRFの情報と、軌道の情報から、2つの点群が最も近いときの距離をまとめて返す関数
    void search_LRF_Traj(sensor_msgs::LaserScan& latest_scan, std::vector<std::vector<std::vector<double>>>& PredictTrajs);

    void clear(){
        LRFpoints.clear();
        LRFkdtree.clear();
        scan_indices.clear();
        traj_indices.clear();
        lin_dists.clear();
        ang_dists.clear();
    };
};

#endif