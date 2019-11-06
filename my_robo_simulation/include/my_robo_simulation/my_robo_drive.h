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

class my_robo : public DWA_var
{
public:
    ros::Subscriber sub_odom;
    ros::Subscriber sub_lrf;
    ros::Subscriber sub_joy;
    ros::Publisher pub_cmd;

    ros::NodeHandle n;

    //　最終的にパブリッシュする速度司令
    geometry_msgs::Twist vel;

    // joyからの速度指令を保存する
    double x_vel_joy;
    double z_ang_joy;

    //　最終的にパブリッシュする速度司令
    geometry_msgs::Twist pub_vel;

    my_robo_spec spec;
    my_robo_sensor sensor;

    // DWA_var DWA;

    ros::Publisher pub_mark;
    ros::Publisher pub_mark_arr;

    my_robo();

    void controlloop();

    void cal_DWA();

    // 予測軌道を計算する関数
    void cal_predict_position();

    // 次の時刻のロボットの位置を計算する関数
    position robot_model(position p_now, double cand_v, double cand_w, double dt);

    // 障害物との入力相当距離(SharedDWAのd_Uにあたる)を求め,Admを返す
    // すべてのLRFの点を用いたバージョン
    // void cal_Dist();

    // LRFの代表点を用いたバージョン
    void cal_Dist2();

    // 評価関数を計算する Dはsat係数.あらかじめ計算しておく
    int cal_J_sharedDWA(double D);

    // 速度コストのsaturation係数を求める
    double cal_vel_sat();

    void DWAloop();

    void clear_vector();

    void check_joy();

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

    // gnuplotで表示
    void plot_d_deg_gnuplot(FILE *gp);

    std::vector<double> LOG;
    std::ofstream logfile;

    double cal_head_cost(int,double);

    double cal_vel_cost(int);

    const void push_back_traj(const int, const double, const position, const double, const double);
};

#endif
