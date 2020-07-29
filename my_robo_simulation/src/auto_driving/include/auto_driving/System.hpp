#include <ros/ros.h>
#include "auto_driving/Goal.hpp"
#include "auto_driving/Image.hpp"
#include "auto_driving/Velocity.hpp"
#include "auto_driving/Scan.hpp"

// #include <iostream>
#include <string>

namespace JetSAS
{
class System
{
private:
    std::string name_node_;
    ros::NodeHandle nh_;

    Goal *goal_ptr_;
    Image *iamge_ptr_;
    Velocity *velocity_ptr_;
    Scan *scan_ptr_;

    float loop_rate_;
    ros::Rate rate_;

public:
    System();

    ~System();

    void set_publisher();


    void mainloop();
};

}