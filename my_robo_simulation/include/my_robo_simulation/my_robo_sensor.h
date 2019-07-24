#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Joy.h"
#include "visualization_msgs/MarkerArray.h"
#include <tf/transform_broadcaster.h>

#include "my_robo_spec.h"

#ifndef MY_ROBO_SENSOR
#define MY_ROBO_SENSOR

#define DEG2RAD M_PI / 180
#define RAD2DEG 180 / M_PI

// nav_msgs::Odometry sodom;
// sensor_msgs::LaserScan slatest_scan;
// sensor_msgs::Joy sjoy;

class my_robo_sensor
{
public:
    // lrfから得られた距離データ
    sensor_msgs::LaserScan latest_scan;

    // オドメトリ情報
    nav_msgs::Odometry odom;

    sensor_msgs::Joy joy;

    //センサの初回実行を確かめる
    int count, countj;

    // [0]が速度[1]が角速度
    double joy_cmd_vel[2];

    // オドメトリを原点とした障害物の位置 [index][x,y]
    std::vector<std::vector<double>> obs;

    // LRFのインデックス計算のための定数
    int center, degpoint, range_point;

    ros::Publisher pub_mark_arr;

    // my_roboのコンストラクタでも行っている
    my_robo_sensor()
    {
        joy_cmd_vel[0] = 0;
        joy_cmd_vel[1] = 0;
    }

    // 正面からth度ずつwth度まで、障害物の位置を計算する
    visualization_msgs::MarkerArray cal_obs(sensor_msgs::LaserScan &scan, double th, double wth, geometry_msgs::PoseWithCovariance &pose);

    void cb_odom(const nav_msgs::Odometry::ConstPtr &msg);

    void cb_lrf(const sensor_msgs::LaserScan::ConstPtr &msg);

    void cb_joy(const sensor_msgs::Joy::ConstPtr &joy_msg);

    visualization_msgs::MarkerArray pub_markers(std::vector<std::vector<double>> obs);
};

#endif