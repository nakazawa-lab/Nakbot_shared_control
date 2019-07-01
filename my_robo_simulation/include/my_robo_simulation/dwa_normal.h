#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"    //odometry
#include"my_robo_simulation/my_robo_spec.h"
#include<vector>
#include<algorithm>

#ifndef DWA_NORMAL
#define DWA_NORMAL

/*
struct pair_vel{
    float linear_vel=0.1;
    float angular_vel=0.05;
};
*/
/*
struct widnow{
    float max_vel,max_ang,min_vel,min_ang;
};
*/

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

/*
static void dwa_normal::calculate_dwa(my_robo_spec spec,float predict_time, float dt,std::vector<pair_vel>& cadidate_vel_list, pair_vel velNow){
    // 車両モデルによるwindow Vs
    // x_max_vel
    // x_min_vel
    // z_max_ang
    // z_min_ang

    // 運動モデルからなるwindow
    window Vd{velNow.linear_vel+spec.x_max_acc*dt, velNow.angular_vel+spec.z_max_acc*dt, velNow.linear_vel+spec.x_min_acc*dt, velNow.angular_vel+spec.z_min_acc*dt};
    
    // ２つのモデルを足し合わせたwindow
    window Vtmp{std::max(Vd.max_vel,x_max_vel), std::max(Vd.max_ang,z_max_ang),
                    std::min(Vd.min_vel,x_min_vel), std::min(Vd.min_ang,z_min_ang)
                     };

    // 速度解像度から、候補となる速度対を計算
    pair_vel tmp{linear_vel=Vtmp.min_vel, angular_vel=Vtmp.min_ang};


    for(float v = tmp.linear_vel; v < Vtmp.max_vel; v += resolution.linear_vel){
        tmp.linear_vel=v;
        for(float w = tmp.angular_vel; w < Vtmp.max_ang; w += resolution.angular_vel){
             tmp.angular_vel=w;
             candidate_vel_list.push_back(tmp);
        }
    }

}
*/

#endif
