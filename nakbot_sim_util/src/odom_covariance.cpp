#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace nakbot_sim_util
{

class CovarianceAdjuster
{
private:
    ros::NodeHandle nh_;

    ros::Subscriber sub_odom_ = nh_.subscribe("/odom", 1, &CovarianceAdjuster::odomCallback, this);
    ros::Publisher pub_odom_ = nh_.advertise<nav_msgs::Odometry>( "/odom/republished", 10);


public:
    CovarianceAdjuster(){  };

    ~CovarianceAdjuster(){  };

    void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
    {
        ROS_INFO("callback");
        nav_msgs::Odometry odom_repub;
        odom_repub.header = odom_msg->header;
        odom_repub.child_frame_id = odom_msg->child_frame_id;
        odom_repub.pose = odom_msg->pose;
        odom_repub.twist = odom_msg->twist;

        pub_odom_.publish(odom_repub);
    }

};




} // namespace nakbot_sim_util

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_covariance_republisher");
    nakbot_sim_util::CovarianceAdjuster cov;

    ros::spin();

    return EXIT_SUCCESS;
}