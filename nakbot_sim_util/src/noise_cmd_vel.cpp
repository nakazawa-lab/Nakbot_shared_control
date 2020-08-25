#include <iostream>
#include <vector>
#include <random>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

namespace nakbot_sim_util
{

class CmdNoise
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber sub_cmd_ = nh_.subscribe("/cmd_vel/raw", 1, &CmdNoise::cmdCallback, this);
    ros::Publisher pub_cmd_ = nh_.advertise<geometry_msgs::Twist>( "/cmd_vel", 10);

    std::random_device seed_gen;
    const double bias_lin = 0.01;
    const double bias_ang = 0.001;


public:
    CmdNoise(){  };

    ~CmdNoise(){  };

    void cmdCallback(const geometry_msgs::TwistConstPtr &cmd_msg)
    {
        ROS_INFO("callback");
        geometry_msgs::Twist cmd_repub;

        cmd_repub.linear  = cmd_msg->linear;
        cmd_repub.angular = cmd_msg->angular;

        std::mt19937 engine(seed_gen());
        std::normal_distribution<double> dist_lin(0.0, 0.1);
        std::normal_distribution<double> dist_ang(0.0, 0.1);

        cmd_repub.linear.x  += bias_lin + dist_lin(engine);
        cmd_repub.angular.z += bias_ang + dist_ang(engine);

        pub_cmd_.publish(cmd_repub);
    }

};


} // namespace nakbot_sim_util

int main(int argc, char** argv)
{
    ros::init(argc, argv, "noise_cmd_vel");
    nakbot_sim_util::CmdNoise cov;

    ros::spin();

    return EXIT_SUCCESS;
}