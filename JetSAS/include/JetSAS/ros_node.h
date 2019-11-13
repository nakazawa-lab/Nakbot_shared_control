#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"

class JetSAS_Node{
private:
    ros::NodeHandle nh;

    ros::Publisher lrf_pub;
    ros::Publisher odom_pub;
    ros::Publisher joy_pub;

    sensor_msgs::LaserScan scan;
    sensor_msgs::Joy joy;
    nav_msgs::Odometry odom;

    int old_encoder_right, old_encoder_left;

    double x,y,theta,v,w;

    float joy_axes1, joy_axes0;     // axes1は前後方向, 前が正, axes0は左右方向, 左が正

    double old_time, this_loop_time;

    const int urg_data_num =682;        // 実機で得られる点の数
    const float urg_angle_increment = (2*M_PI)/1024.0;

public:
    JetSAS_Node(){
        scan.ranges.resize(urg_data_num);
        scan.angle_increment = urg_angle_increment;

        lrf_pub = nh.advertise<sensor_msgs::LaserScan>("/scan",1);
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom",1);
        joy_pub = nh.advertise<sensor_msgs::Joy>("/joy",1);
    }

    void pub_lrf(){
        lrf_pub.publish(scan);
    };

    void pub_odom(){
        odom_pub.publish(odom);
    };

    void pub_joy(){
        joy_pub.publish(joy);
    };
    
    void make_scan_msgs(long*,const int);

    void make_joy_msgs();

    void make_odom_msgs();

    void controlloop();
};
