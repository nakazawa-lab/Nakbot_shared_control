#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"    
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"

#include"my_robo_spec.h"

#ifndef MY_ROBO_SENSOR
#define MY_ROBO_SENSOR

#define DEG2RAD M_PI/180
#define RAD2DEG 180/M_PI

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

    // オドメトリを原点とした障害物の位置 [index][x,y]
    std::vector<std::vector<float>> obs;

    // LRFのインデックス計算のための定数
    int center,degpoint,range_point;

// my_roboのコンストラクタでも行っている
    my_robo_sensor(){
        joy_cmd_vel[0]=0;
        joy_cmd_vel[1]=0;
    }

    // 正面からth度ずつwth度まで、障害物の位置を計算する
    void cal_obs(sensor_msgs::LaserScan& scan, float th, float wth,float x,float y){
        // th 度相当の点の数degpを求める
        int thp = (int)((th * DEG2RAD)/latest_scan.angle_increment);
        //ROS_INFO("degp:%d",thp);
        int deg_inc = (int)(wth/th);

        //ROS_INFO("deg_inc:%d",deg_inc);
        for(int i = -deg_inc ;i< deg_inc ; i++){
            if(scan.ranges[center+thp*i] < scan.range_max && scan.ranges[center+thp*i] > scan.range_min){
            obs.push_back(std::vector<float>());
            float xo = x +  scan.ranges[center+thp*i]*cos(th*DEG2RAD*i);
            float yo = y +  scan.ranges[center+thp*i]*sin(th*DEG2RAD*i);
            
            obs.back().push_back(xo);
            obs.back().push_back(yo);
            //ROS_INFO("obs x:%f, y:%f",xo,yo);
         }
        }

    }

    void cb_odom(const nav_msgs::Odometry::ConstPtr &msg)
    {
        // ROS_INFO("odom vel %f", msg->twist.twist.linear.x);
	    odom = *msg;
    }

   void cb_lrf(const sensor_msgs::LaserScan::ConstPtr &msg){
        // 受け取ったメッセージをコピーしておく
        latest_scan = *msg;
        if(count==0){
            ROS_INFO("first laser scan.");
            range_point=latest_scan.ranges.size();                // 取得した点の数
            center=range_point/2;
            ROS_INFO("range_point:%d.",range_point);
            ROS_INFO("center:%d.",center);
            count++;   
        }
        // <TODO>>
        // 測定範囲外の場合の対応
    }


//    void cb_joy(const sensor_msgs::Joy& joy_msg, float joy_cmd_vel[2], my_robo_spec spec){
//         // ジョイスティック左側
//         // 上→axes[1]の正方向
//         // 左→axes[0]の正方向
//         joy_cmd_vel[0] =spec.x_max_vel*joy_msg.axes[1];
//         //cmd_vel.linear.y =joy_msg.axes[2];

//         if(joy_msg.axes[1]>=0) joy_cmd_vel[1]=spec.z_max_ang*joy_msg.axes[0];

//         else joy_cmd_vel[1] = -1*spec.z_max_ang*joy_msg.axes[0];

//         ROS_INFO("x_joy: %f",joy_cmd_vel[0]);
//         ROS_INFO("z_ang: %f\n",joy_cmd_vel[1]);
//     }

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
        //ROS_INFO("count j:%d",countj);
    }

};


#endif