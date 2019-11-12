#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

class JetSAS_Node{
private:
    ros::NodeHandle nh;
    ros::Publisher lrf_pub;
    sensor_msgs::LaserScan *scan;
public:
    JetSAS_Node(){
        lrf_pub = nh.advertise<sensor_msgs::LaserScan>("/scan",1);
    }

    void pub_lrf();
    
    void make_sensor_msgs(long*);
};