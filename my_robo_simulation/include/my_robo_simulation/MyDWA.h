#include "my_robo_simulation/my_robo_drive.h"

#ifndef MY_DWA
#define MY_DWA

class MyDWA : public my_robo
{
private:
    // distを計算せず、直接d thのcostを計算していくときに保持するコスト
    std::vector<std::vector<double>> dist_lin_ang;

    std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> loop_start_time;
    std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> cal_end_time;

    void cal_opt();
    void kd_tree();    

    visualization_msgs::MarkerArray make_traj_marker_array(int);
    visualization_msgs::MarkerArray make_joy_traj_marker_array();
    visualization_msgs::Marker make_nearest_LRF_marker(float, float);

    void plot_gnuplot(FILE *gp);
    void plot_scan_gnuplot(FILE *gp,std::vector<float>&,std::vector<float>&);

    void Proposed();   

    void log_init();
    void record_param();
    void record_loop_info();
    void close_file();

public:
    MyDWA(){};

    void DWAloop();

    void clear_vector();
    
    std::ofstream mylogfile;

    bool IsProposed;

};

#endif