#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"    //odometry
#include <tf/transform_broadcaster.h>
#include "visualization_msgs/MarkerArray.h"

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

#endif