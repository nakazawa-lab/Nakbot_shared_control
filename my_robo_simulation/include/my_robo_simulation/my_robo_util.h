#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"    //odometry
#include "tf/transform_broadcaster.h"
#include "visualization_msgs/MarkerArray.h"
#include "matplotlibcpp.h"

#include<chrono>
#include<vector>
#include<cmath>

#ifndef MY_ROBO_UTIL
#define MY_ROBO_UTIL

#define DEG2RAD M_PI / 180
#define RAD2DEG 180 / M_PI

double add_theorem_sin(double,double,double,double);

double add_theorem_cos(double,double,double,double);

void geometry_quat_to_rpy(double&,double&,double&,geometry_msgs::Quaternion);

visualization_msgs::MarkerArray make_markers_2Dvector(std::vector<std::vector<double>> obs);

double cal_average_d_U(std::vector<std::vector<double>>& CandVel);

double cal_euclid(double x0, double y0, double x1, double y1);

void say_time(const char *name, std::chrono::time_point<std::chrono::_V2::system_clock,std::chrono::nanoseconds> basetime);

#endif