#include <math.h>
#include <cassert>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"

#ifndef ROS_NODE
#define ROS_NODE

namespace JetSAS
{
/// ==TODO== ///
const double robot_width = 0.3;

struct position
{
    double x = 0;
    double y = 0;
    double sin_th = 0;
    double cos_th = 1;
};

struct Serial_sh{
    struct Rc{
        int rot, lin, chan3, chan4;
    };
    struct Encoder{
        int r_ref,l_ref,r_sum,l_sum;
    };

    Rc rc;
    Encoder encoder;
};

class Lrf
{
private:
    ros::Publisher lrf_pub;
    sensor_msgs::LaserScan scan;

    const int urg_data_num = 682; // 実機で得られる点の数
    const float urg_angle_increment = (2 * M_PI) / 1024.0;

    float scan_calib_multiplier;

public:
    Lrf()
    {
        scan.ranges.resize(urg_data_num);
        scan.angle_increment = urg_angle_increment;
    }

    void set_publisher(ros::NodeHandle &nh)
    {
        lrf_pub = nh.advertise<sensor_msgs::LaserScan>("/scan", 1);
    };

    void pub_lrf()
    {
        lrf_pub.publish(scan);
    };

    void make_scan_msgs(long *, const int);
};

class Joy
{
private:
    ros::Publisher joy_pub;
    sensor_msgs::Joy joy;

    float joy_axes1, joy_axes0; // axes1は前後方向, 前が正, axes0は左右方向, 左が正

    float axes1_multiplier, axes0_multiplier;

public:
    Joy()
    {
    }

    void set_publisher(ros::NodeHandle &nh)
    {
        joy_pub = nh.advertise<sensor_msgs::Joy>("/joy", 1);
    };

    void pub_joy()
    {
        joy_pub.publish(joy);
    };

    // ラジコンの値を人間の指令の(v,w)に変換...はしなくてよい
    // 並進方向の大きさをjoy_axes1に, 回転方向の大きさをjoy_axes0に, 最大最小が+-1になるように入れてあげてpubすれば良い
    void make_joy_msgs(int, int);
};

class Odom
{
private:
    ros::Publisher odom_pub;
    nav_msgs::Odometry odom;

    int encoder_right, encoder_left;
    int old_encoder_right, old_encoder_left;

    const double encoder_multiplier_vel = 0.1;

    double v, w;

    double right_v_from_enc, left_v_from_enc;
    double v_from_enc, w_from_enc;

    double right_v, left_v;

    position now_p, old_p;

    void set_encoder(const int e_right, const int e_left)
    {
        old_encoder_right = encoder_right;
        old_encoder_left = encoder_left;

        encoder_right = e_right;
        encoder_left = e_left;
    };

    void cal_now_vel(const double this_loop_time);

    void cal_pose(double dt);

public:
    Odom()
    {
    }

    void set_publisher(ros::NodeHandle &nh)
    {
        odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 1);
    }

    void pub_odom()
    {
        odom_pub.publish(odom);
    };

    void make_odom_msgs(const int, const int, const double);

    // エンコーダの値から今のv,wを求める
    // 昔のx,yの情報と, 今のv,wとこの1ループの時間から新しい位置x,y を求める

    // set_encoder → cal_now_vel → cal_pose
};

class Cmd_vel
{
private:
    ros::Subscriber sub_vel;
    geometry_msgs::Twist vel;

    int encoder_prm_r, encoder_prm_l;

    const double cmd_multipler_vel = 0.1;
    const double cmd_multiplier_rot = 0.1;


public:
    Cmd_vel()
    {
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
    }

    void set_subscriber(ros::NodeHandle &nh)
    {
        sub_vel = nh.subscribe("/cmd_vel", 10, &Cmd_vel::cb_vel, this);
    }

    void cb_vel(const geometry_msgs::Twist::ConstPtr &msgs)
    {
        vel = *msgs;
    }

    void cmd_vel_to_encoder();
};

class RC{
private:
    const double rc_multiplier_vel= 0.1;
    const double rc_multiplier_rot= 0.1;
    double v_h,w_h;
public:
    RC(){

    }
    void rc_to_vel();
};
} // namespace JetSAS

class JetSAS_Node
{
private:
    ros::NodeHandle nh;
    double old_time=0.0;
    double this_loop_time;

    void pub_sensor()
    {
        lrf.pub_lrf();
        odom.pub_odom();
        joy.pub_joy();
    };

public:
    JetSAS_Node()
    {
        lrf.set_publisher(nh);
        odom.set_publisher(nh);
        joy.set_publisher(nh);
    }

    JetSAS::Lrf lrf;

    JetSAS::Odom odom;

    JetSAS::Joy joy;

    JetSAS::Cmd_vel cmd_vel;

    JetSAS::RC rc;

    void controlloop(JET_TIMER);
};

extern JetSAS::Serial_sh ros_serial;

#endif /// ROS_NODE