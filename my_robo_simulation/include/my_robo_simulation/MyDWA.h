#include "my_robo_simulation/my_robo_drive.h"

#ifndef MY_DWA
#define MY_DWA

class MyDWA : public my_robo
{
private:
    std::vector<int> scan_indices, traj_indices;

    std::vector<double> lin_normdists, ang_normdists;

    // 小さいと、その次元の距離を実際の距離より小さくみつもることになり、その次元は危険とみなされる
    // 小さいとその次元の距離が短くなる
    //const double point_scale_d = 1;
    //const double point_scale_th = 1;        // maxang/maxvel * 2 = 3.75

    // distを計算せず、直接d thのcostを計算していくときに保持するコスト
    std::vector<std::vector<double>> dist_lin_ang;

    std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> loop_start_time;

    std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> cal_end_time;

    // std::vector<float> thinout_scan_range;

    // std::vector<float> thinout_scan_ang;

    // treeidxのツリーのidx番目の点と、queryの点の距離(を求める。
    // double cal_Dist(MyPoint query, int idx);

    void cal_opt();

    void kd_tree();

    void record_param();

    //void make_mylog(double,double,double,double,double,double,double,int);

    //void make_mylog_perloop(double);

    visualization_msgs::MarkerArray make_traj_marker_array(int);

    visualization_msgs::MarkerArray make_joy_traj_marker_array();

    void plot_gnuplot(FILE *gp);
    void plot_scan_gnuplot(FILE *gp,std::vector<float>&,std::vector<float>&);

    void Proposed();

    visualization_msgs::Marker make_nearest_LRF_marker(float, float);

    void record_loop_info();

public:
    MyDWA(){};

    void DWAloop();

    void clear_vector();
    
    std::ofstream mylogfile;

    bool IsProposed;

};

#endif