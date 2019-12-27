#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h" //odometry
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"

#include <vector>
#include <cmath>
#include <fstream>

// 自作クラス
#include "my_robo_simulation/my_robo_spec.h"
#include "my_robo_simulation/my_robo_sensor.h"
#include "my_robo_simulation/DWA_var.h"

#ifndef MY_ROBO_DRIVE
#define MY_ROBO_DRIVE

class my_kd_tree{
public:
    kdt::KDTree<MyPoint> LRFkdtree;
    std::vector<MyPoint> LRFpoints;
    std::vector<float> thinout_scan_x;
    std::vector<float> thinout_scan_y;
};

class my_robo : public DWA_var, public my_kd_tree
{
public:
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

        double dist, adm;
    };
    selected_vel_info selected;

    ros::Subscriber sub_odom;
    ros::Subscriber sub_lrf;
    ros::Subscriber sub_joy;
    ros::Publisher pub_cmd;
    ros::NodeHandle n;

    //　最終的にパブリッシュする速度司令
    geometry_msgs::Twist vel;

    // 最終的に採用する軌道のインデックス
    int opt_index;

    my_robo_spec spec;
    my_robo_sensor sensor;

    ros::Publisher pub_mark;
    ros::Publisher pub_mark_arr;

    std::vector<double> LOG;
    std::ofstream logfile;

    my_robo();
    void controlloop();
    void cal_DWA();
    
    void cal_predict_position();        // 予測軌道を計算する関数
    void cal_Dist2();
    double cal_head_cost(int,double);
    double cal_vel_cost(int);
    void cal_J_sharedDWA(double D);     // 評価関数を計算する Dはsat係数.あらかじめ計算しておく
    double cal_vel_sat();               // 速度コストのsaturation係数を求める

    void DWAloop();
    void check_joy();
    void clear_vector();

    visualization_msgs::Marker make_pos_marker(position p);

    void pub_marker_array(visualization_msgs::MarkerArray markers)
    {
        pub_mark_arr.publish(markers);
    };

    void pub_marker(visualization_msgs::Marker marker)
    {
        pub_mark.publish(marker);
    };

    position cal_nowp(nav_msgs::Odometry &odom);
    const void push_back_traj(const int, const double, const position, const double, const double);

    // 次の時刻のロボットの位置を計算する関数
    position robot_model(position p_now, double cand_v, double cand_w, double dt);

    // gnuplotで表示
    void plot_d_deg_gnuplot(FILE *gp);
};

#endif
