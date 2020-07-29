#include "ros/ros.h"
#include "geometry_msgs/Twist.h" 

#include <string>

namespace JetSAS
{
class Velocity
{
private:
    const std::string topic_name_vel_;

    geometry_msgs::Twist cmd_vel_;
    ros::Publisher publisher_vel_;


public:
    Velocity();
    ~Velocity();

    void update_velocity();
    void make_stay_vel();
    void publish_vel();

};


}