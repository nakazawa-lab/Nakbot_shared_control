#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"    //odometry
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"

#include"my_robo_spec.h"

#ifndef MY_ROBO_SENSOR
#define MY_ROBO_SENSOR

// nav_msgs::Odometry sodom;
// sensor_msgs::LaserScan slatest_scan;
// sensor_msgs::Joy sjoy;

class my_robo_sensor{
public:
    // lrfから得られた距離データ
    sensor_msgs::LaserScan latest_scan;

    // オドメトリ情報
    nav_msgs::Odometry odom;

    sensor_msgs::Joy joy;

    //センサの初回実行を確かめる
    int count,countj;    

    // [0]が速度[1]が角速度
    float  joy_cmd_vel[2];


    void cb_odom(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // ROS_INFO("odom vel %f", msg->twist.twist.linear.x);
	    odom = *msg;
    }

   void cb_lrf(const sensor_msgs::LaserScan::ConstPtr &msg){
        // 受け取ったメッセージをコピーしておく
        latest_scan = *msg;
        if(count==0)ROS_INFO("first laser scan.");
        
        // <TODO>>
        // 測定範囲外の場合の対応
    }


   void cb_joy(const sensor_msgs::Joy& joy_msg, float joy_cmd_vel[2], my_robo_spec spec){
        // ジョイスティック左側
        // 上→axes[1]の正方向
        // 左→axes[0]の正方向
        joy_cmd_vel[0] =spec.x_max_vel*joy_msg.axes[1];
        //cmd_vel.linear.y =joy_msg.axes[2];

        if(joy_msg.axes[1]>=0) joy_cmd_vel[1]=spec.z_max_ang*joy_msg.axes[0];

        else joy_cmd_vel[1] = -1*spec.z_max_ang*joy_msg.axes[0];

        ROS_INFO("x_joy: %f",joy_cmd_vel[0]);
        ROS_INFO("z_ang: %f\n",joy_cmd_vel[1]);
    }

    void cb_joy(const sensor_msgs::Joy::ConstPtr &joy_msg){
        // // ジョイスティック左側
        // // 上→axes[1]の正方向
        // // 左→axes[0]の正方向
        // joy_cmd_vel[0] =spec.x_max_vel*joy_msg.axes[1];
        // //cmd_vel.linear.y =joy_msg.axes[2];

        // if(joy_msg.axes[1]>=0) joy_cmd_vel[1]=spec.z_max_ang*joy_msg.axes[0];

        // else joy_cmd_vel[1] = -1*spec.z_max_ang*joy_msg.axes[0];

        // ROS_INFO("x_joy: %f",joy_cmd_vel[0]);
        // ROS_INFO("z_ang: %f\n",joy_cmd_vel[1]);

        // メッセージを保管しておく
        joy = *joy_msg;

        if(countj==0) {
            ROS_INFO("first joy sub.");
            countj++;
        }
    }

};


#endif