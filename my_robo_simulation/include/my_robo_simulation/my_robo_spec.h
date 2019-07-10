#include "ros/ros.h"

#ifndef MY_ROBO_SPEC
#define MY_ROBO_SPEC


class my_robo_spec
{
public:
    // ロボットの最大速度,加速度
    float x_max_vel;
    float z_max_ang;

    float x_min_vel;
    float z_min_ang;

    float x_max_acc;
    float x_min_acc;

    float z_max_acc;
    float z_min_acc;

    // ロボットの速度、加速度の分解能
    float vel_res = 0.1;
    float ang_res = 0.18;    // 10degくらい

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

        //ROS_INFO("finish getparam:max_x %f",spec.x_max_vel);

        //return spec;
    }

    void set_resolution(float Vres,float Wres){
        vel_res = Vres;
        ang_res = Wres;
    }

    // コンストラクタ
    // my_robo_spec(ros::NodeHandle n,float Vres,float Wres ){
    //     get_spec_param(n,);
    //     set_resolution(Vres,Wres);
    // }

public:
    my_robo_spec(){
    }
};

#endif
