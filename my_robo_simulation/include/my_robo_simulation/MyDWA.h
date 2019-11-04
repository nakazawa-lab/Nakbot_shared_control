#include "my_robo_simulation/kdtree.h"
#include "my_robo_simulation/my_robo_drive.h"

#ifndef MY_DWA
#define MY_DWA

// kdtree.hの利用のための、点を表すクラス
class MyPoint : public std::array<double, 2>
{
public:
    static const int DIM = 2;

    MyPoint() {}
    MyPoint(double x, double y)
    {
        (*this)[0] = x;
        (*this)[1] = y;
    }
};

class MyDWA : public my_robo
{
private:
    std::vector<int> scan_indices, traj_indices;

    std::vector<double> lin_normdists, ang_normdists;

    // 小さいと、その次元の距離を実際の距離より小さくみつもることになり、その次元は危険とみなされる
    // 小さいとその次元の距離が短くなる
    //const double point_scale_d = 1;
    //const double point_scale_th = 1;        // maxang/maxvel * 2 = 3.75

    // 最終的に採用する軌道のインデックス
    int opt_index;

    bool IsProposed;

    struct selected_vel_info
    {
    public:
        double linadm;
        double linsafe;
        double angadm;
        double angsafe;

        double vel, ang;
        double vel_h_cost;
        double head_h_cost;
        double cost;

        double lindist,angdist;
    };
    selected_vel_info selected;

    // LRFのkd木
    kdt::KDTree<MyPoint> LRFkdtree;

    // LRFについてのPoints
    std::vector<MyPoint> LRFpoints;

    // distを計算せず、直接d thのcostを計算していくときに保持するコスト
    std::vector<std::vector<double>> dist_lin_ang;

    std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> loop_start_time;

    std::vector<float> thinout_scan_range;

    std::vector<float> thinout_scan_ang;

    // DWA_var DWA;

    // treeidxのツリーのidx番目の点と、queryの点の距離(を求める。
    double cal_Dist(MyPoint query, int idx);

    double cal_head_cost_pro(int);

    double cal_vel_cost_pro(int);

    void cal_opt();

    void kd_tree();

    void record_param();

    //void make_mylog(double,double,double,double,double,double,double,int);

    //void make_mylog_perloop(double);

    visualization_msgs::MarkerArray make_traj_marker_array(int);

    void plot_gnuplot(FILE *gp);

    void Proposed();

    visualization_msgs::Marker make_nearest_LRF_marker(int optId);

public:
    MyDWA(){};

    void DWAloop();

    void clear_vector();
    
    std::ofstream mylogfile;


};

#endif