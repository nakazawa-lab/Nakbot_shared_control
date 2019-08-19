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


// 検出した線に関する情報を集めたクラス
class line{
private:
    int start_point_index;
    int last_point_index;

    int num_point;

    double curve;

    // ロボット座標系から見た線のはじめのX座標とY座標
    double start_X,start_Y,end_X,end_Y;

public:
    line(int s,int e,const sensor_msgs::LaserScan &scan){
        start_point_index = s;
        last_point_index = e;
        num_point = e - s + 1;
        cal_curve(start_point_index,last_point_index,scan);
    };

    // この中でstar_Xなどが決定され代入される
    double cal_curve(int startPoint, int endPoint, const sensor_msgs::LaserScan &scan);

    double cal_ang_fromfront(int index, const sensor_msgs::LaserScan &scan);

    visualization_msgs::MarkerArray make_edge_marker(int center,const sensor_msgs::LaserScan &scan, geometry_msgs::PoseWithCovariance &pose);
};

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

    // 検出された線についての情報
    std::vector<line> lines;

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

    visualization_msgs::MarkerArray make_obs_markers(std::vector<std::vector<double>> obs);

    void pub_joy_marker();

    void detect_line(const sensor_msgs::LaserScan &scan);
    
    static void next_is_line(int point, double th);
};



#endif