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
    float dt = 0.05;
    float PredictTime = 3;
    float k_heading = 0.1;
    float k_velocity = 0.1;

    // 予測軌道 [index][時刻,x,y,theta]
    std::vector<std::vector<float>>  PredictTraj;

    // 候補となる(v,w)の対
    std::vector<std::vector<float>> CandVel;
};

// 次の位置を格納する箱
struct position{
    float x;
    float y;
    float th;
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
    //std::vector<std::vector<float>> spec;
    // →specに統合

    // joyからの速度指令を保存する
    float x_vel_joy;
    float z_ang_joy;

    //　最終的にパブリッシュする速度司令
    geometry_msgs::Twist pub_vel;


    my_robo_spec spec;
    my_robo_sensor sensor;

    DWA_var DWA;

    // 次の位置を格納する箱






public:
    my_robo();
    //~my_robo();

    void controlloop();

    void cal_DWA(my_robo_spec& spec, my_robo_sensor& sensor, DWA_var& DWA);

    void cal_J_DWA(){

    }

    void cal_sharedDWA_J(){

    }

    // 予測軌道を計算する関数
    void cal_predict_position(DWA_var& DWA, my_robo_spec spec, std::vector<std::vector<float>> CandVel,
      std::vector<std::vector<float>>&  PredictTraj, my_robo_sensor& sensor);


    // 次の時刻を計算する関数
    position robot_model( position p_now, float cand_v , float cand_w , float dt,  my_robo_sensor& sensor){
        position p_next;
        p_next.x = p_now.x +  cand_v * cos(p_now.th) * dt * cos( p_now.th + dt/2);
        p_next.y = p_now.y  + cand_v * sin(p_now.th) * dt * sin( p_now.th + dt/2);;
        p_next.th = p_now.th + cand_w * dt;
        return p_next;
    }


    // joyの速度司令との差を計算する関数

    // 
};

#endif


