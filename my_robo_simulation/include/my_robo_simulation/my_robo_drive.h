#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"    //odometry
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"

#include<vector>
#include<cmath>

// 自作クラス
#include"my_robo_simulation/my_robo_spec.h"
#include"my_robo_simulation/my_robo_sensor.h"



#ifndef MY_ROBO_DRIVE
#define MY_ROBO_DRIVE



    // DWAのセッティング
struct DWA_var{
    // DWA設定の刻み.ループレイトと同じが望ましい
    double dt =0.25;
    // 軌道計算の刻み
    double dt_traj=0.2;
    // 軌道予測時刻
    double PredictTime = 3 ;
    //double looprate = 2;          // Hz
    double looprate = 1 / dt;

    double k_heading = 1;
    double k_velocity = 1;


    // 予測軌道 [index][時刻index][time,x,y,sin cos]
    std::vector<std::vector<std::vector<double>>>  PredictTraj;

    std::vector<std::vector<double>> Joy_PredictTraj;

    // 候補となる(v,w)の対
    // [index][v,w,d_U]
    std::vector<std::vector<double>> CandVel;

    // 衝突判定用フラグ
    std::vector<bool> isCollision;
};

// 次の位置を格納する箱
struct position{
    double x;
    double y;
    double sin_th;
    double cos_th;
};

class my_robo{
private:
    ros::Subscriber sub_odom;
    ros::Subscriber sub_lrf;
    ros::Subscriber sub_joy;
    ros::Publisher pub_cmd;

    ros::NodeHandle n;



    //　最終的にパブリッシュする速度司令
    geometry_msgs::Twist vel;              

    // ロボットの最大速度,加速度 コンストラクタでパラメータから読み込む
    // [i][0]速度 [i][1]角速度
    // i=0 max vel  1 min vel   2 max acc     3 min acc
    //std::vector<std::vector<double>> spec;
    // →specに統合

    // joyからの速度指令を保存する
    double x_vel_joy;
    double z_ang_joy;

    //　最終的にパブリッシュする速度司令
    geometry_msgs::Twist pub_vel;


    my_robo_spec spec;
    my_robo_sensor sensor;

    DWA_var DWA;

    ros::Publisher pub_mark;
    ros::Publisher pub_mark_arr;

    // 次の位置を格納する箱


public:
    my_robo();
    //~my_robo();

    void controlloop();

    void cal_DWA();

    void cal_J_DWA(){

    }

    // 予測軌道を計算する関数
    void cal_predict_position();


    // 次の時刻のロボットの位置を計算する関数
    position robot_model( position p_now, double cand_v , double cand_w , double dt);

    // 障害物との入力相当距離(SharedDWAのd_Uにあたる)を求め,Admを返す
    // すべてのLRFの点を用いたバージョン
    void cal_Dist();

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

    visualization_msgs::MarkerArray make_traj_marker_array(int index);

    void pub_marker_array(visualization_msgs::MarkerArray markers){
        pub_mark_arr.publish(markers);
    };

    void pub_marker(visualization_msgs::Marker marker){
        pub_mark.publish(marker);
    };
    
    position cal_nowp(nav_msgs::Odometry& odom);

    void say_log();
};

#endif


