#include "ros/ros.h"
#include<tf/transform_broadcaster.h>
#include "geometry_msgs/Twist.h"    
#include "nav_msgs/Odometry.h"

#include "my_robo_simulation/my_robo_util.h"

#ifndef MY_ROBO_SPEC
#define MY_ROBO_SPEC


class my_robo_spec
{
public:
    // ロボットの最大速度,加速度
    double x_max_vel;
    double z_max_ang;

    double x_min_vel;
    double z_min_ang;

    double x_max_acc;
    double x_min_acc;

    double z_max_acc;
    double z_min_acc;

    // ロボットの縦と横の長さ これにより衝突半径を計算する
    const float ROBOT_WIDTH = 0.3;
    const float ROBOT_LENGTH = 0.3;
    const float ROBOT_RAD = sqrt((ROBOT_WIDTH * ROBOT_WIDTH)/4 + (ROBOT_LENGTH* ROBOT_LENGTH)/4);        // ロボットの衝突判定となる円 0.35と0.3だと0.230くらい

    // ロボット自身の大きさを保持するもの
    std::vector<std::vector<float>> RobotSize;

    // ロボットの速度、加速度の分解能
    double vel_res;
    double ang_res;    // 10degくらい

    // スペックをパラメータから取得する関数
    void get_spec_param(ros::NodeHandle n, my_robo_spec& spec){
        //my_robo_spec spec;

        // ロボットの最大速度,最小速度を示すパラメータを読み込む
        n.getParam("/my_robo/diff_drive_controller/linear/x/max_velocity",spec.x_max_vel);
        n.getParam("/my_robo/diff_drive_controller/angular/z/max_velocity",spec.z_max_ang);
        n.getParam("/my_robo/diff_drive_controller/linear/x/min_velocity",spec.x_min_vel);
        n.getParam("/my_robo/diff_drive_controller/angular/z/min_velocity",spec.z_min_ang);

        // ロボットの最大加速度を示すパラメータを読み込む
        n.getParam("/my_robo/diff_drive_controller/linear/x/max_acceleration",spec.x_max_acc);
        n.getParam("/my_robo/diff_drive_controller/angular/z/max_acceleration",spec.z_max_acc);
        n.getParam("/my_robo/diff_drive_controller/linear/x/min_acceleration",spec.x_min_acc);
        n.getParam("/my_robo/diff_drive_controller/angular/z/min_acceleration",spec.z_min_acc);
    }

    void set_resolution(double Vres,double Wres){
        vel_res = Vres;
        ang_res = Wres;
    }

public:
    my_robo_spec(){
        // RobotSize[index][d,theta] thetaは1度ずつ、dはロボットの半径。1度ずつ全周にわたる点群を作成する
        // d-thetaでのロボットサイズの描画に用いている
        for(int i=0;i<360;i++){
            RobotSize.push_back(std::vector<float>());
            RobotSize[i].push_back(ROBOT_RAD);
            RobotSize[i].push_back((i-180)*DEG2RAD);
        }
    }
};

#endif
