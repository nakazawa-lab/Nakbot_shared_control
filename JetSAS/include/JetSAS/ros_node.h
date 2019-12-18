#include <math.h>
#include <cassert>
#include <fstream>
#include <string>
#include <chrono>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"

#ifndef ROS_NODE
#define ROS_NODE

extern std::string get_current_time();

namespace JetSAS
{
const double INTERCEPT_ENCODER = 5000000.0;
const double WHEEL_LENGTH = 0.0254*6.0*M_PI;    // 6インチ径のタイヤの長さ
const double ENCODER_PER_ROT = 5120.0;
const double ENCODER_VEL_DIVIDER = 100.0;
extern const double robot_width;

// 予備実験によってだいたいの値を書いておく
const float max_rc_lin = 1237.0;
const float max_rc_rot = 1234.0;
const float min_rc_lin = 1134.0;
const float min_rc_rot = 1136.0;

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
        int r_init, l_init;
    };
    Rc rc;
    Encoder encoder;

    Serial_sh(){
    rc.rot = 0;
rc.lin = 0;
rc.chan3 = 0;
rc.chan4 = 0;
encoder.r_ref = 0;
encoder.l_ref = 0;
encoder.r_sum = 0;
encoder.l_sum = 0;    
}
};

class Lrf
{
private:
    ros::Publisher lrf_pub;
    sensor_msgs::LaserScan scan;

    const int urg_data_num = 682; // 実機で得られる点の数
    const float urg_angle_increment = (2 * M_PI) / 1024.0;

    float scan_calib_multiplier;
    long long seq_=0;

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
        joy.axes.resize(2);
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
    void make_joy_msgs();
};

class Odom
{
private:
    ros::Publisher odom_pub;
    nav_msgs::Odometry odom;

    int encoder_right=0, encoder_left=0;
    int old_encoder_right=0, old_encoder_left=0;
    int encoder_right_ref=0,encoder_left_ref=0;

    // 5000000が基本
    const double encoder_multiplier = WHEEL_LENGTH / (ENCODER_PER_ROT/ENCODER_VEL_DIVIDER) ;

    double v, w;
    double right_v, left_v;

    position now_p, old_p;

    void set_encoder(const int e_right, const int e_left);

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
    
    bool check_new_encoder();

    // エンコーダの値から今のv,wを求める
    // 昔のx,yの情報と, 今のv,wとこの1ループの時間から新しい位置x,y を求める

    // set_encoder → cal_now_vel → cal_pose
};

class Cmd_vel
{
private:
    ros::Subscriber sub_vel;
    geometry_msgs::Twist vel;

    int encoder_cmd_r, encoder_cmd_l;

    // 5000を基準にしている
    const double cmd_multipler_vel = 0.1;
    const double cmd_multiplier_rot = 0.1;

    const double cmd_multiplier_to_enc = ENCODER_PER_ROT / (WHEEL_LENGTH*10);


public:
    Cmd_vel()
    {
        vel.linear.x = 0.0;
        vel.angular.z = 0.0;
    }

    void set_subscriber(ros::NodeHandle &nh)
    {
        sub_vel = nh.subscribe("/cmd_vel", 1, &Cmd_vel::cb_vel, this);
        std::cout << "set sub" <<std::endl;
    }

    void cb_vel(const geometry_msgs::Twist::ConstPtr &msgs)
    {
        vel = *msgs;
std::cout << std::endl;
        std::cout << "sub cmd" << std::endl;
std::cout << std::endl;
    }

    void cmd_vel_to_encoder();
};

class RC{
private:
    // 1100くらいに基準がある
    const double rc_multiplier_vel_r= -0.5910;
    const double vel_r_int = 703.4;
    const double rc_multiplier_vel_l= -0.5891;
    const double vel_l_int = 698.1;
    const double rc_multiplier_rot_r= -0.5876;
    const double rot_r_int = 702.1;
    const double rc_multiplier_rot_l= 0.5883;
    const double rot_l_int = -697.8;
    double rc_rot;

    double v_right_enc,v_left_enc;      // RCの指令値をエンコーダ換算した値
public:
    RC(){

    }

    void set_rc(const double rc){
        rc_rot = rc;
    };

    void rc_to_encoder();

    bool check_new_rc();
};
} // namespace JetSAS

class JetSAS_Node
{
private:
    ros::NodeHandle nh;
    double old_time=0.0;
    //double this_loop_time;
    std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> start_time;
    std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> last_cal_time;
    double cal_time;

    std::ofstream logfile;
    int counter;
    //std::vector<double> LOG;

    void pub_sensor()
    {
        lrf.pub_lrf();
        odom.pub_odom();
        joy.pub_joy();
    };

    void make_log_col();

    void write_log();

    void register_vel_param();

    // void clear_vector(){
    //     std::vector<double>().swap(LOG);
    // };

public:
    JetSAS_Node()
    {
        lrf.set_publisher(nh);
        odom.set_publisher(nh);
        joy.set_publisher(nh);
        cmd_vel.set_subscriber(nh);
        std::cout << "Jetsas constructor" << std::endl; 
        logfile.open("./log_JetSAS/log_"+get_current_time()+".csv");
        make_log_col();
        start_time = std::chrono::system_clock::now();
        last_cal_time = start_time;
    };

    ~JetSAS_Node(){
        std::cout << "JetSAS destructor" <<std::endl;
        logfile.close();
    };

    JetSAS::Lrf lrf;

    JetSAS::Odom odom;

    JetSAS::Joy joy;

    JetSAS::Cmd_vel cmd_vel;

    JetSAS::RC rc;

    void controlloop(JET_TIMER&);
};
extern JetSAS::Serial_sh ros_serial;
#endif /// ROS_NODE
