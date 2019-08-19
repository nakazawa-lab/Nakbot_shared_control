#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"    //odometry
#include"my_robo_simulation/my_robo_spec.h"
#include<vector>
#include<algorithm>

#ifndef DWA_NORMAL
#define DWA_NORMAL



class dwa_normal{
private:
    // dynamic Window の予測時間と刻み時間
    float predict_time=0.5;
    float dt=0.05;

    // ロボットのスペック、速度解像度
    //my_robo_spec spec;
    float vel_resolution=0.1;
    float ang_resolution=0.05;

    // window [0][0]max_vel [0][1] max_ang [1][0]min_vel [1][1]min_ang
    float window[2][2];

    // 速度の対(v,w)
    float pair_vel[2];

    // 評価関数のパラメータ
    float k_head;
    float k_vel;
    float k_obs;

    // 現在の速度とスペック上の加速度からなる制約を求め,それをみたす候補を返す
    // 車両モデルのwindowと、運動モデルによるwindow
    //static void calculate_dwa(my_robo_spec spec,float predict_time, float dt,std::vector<pair_vel>& cadidate_vel_list, pair_vel velNow);

    // dwaの計算の結果出てくる候補の速度の対
    std::vector<std::vector<float>> cadidate_vel_list;



};
#endif
