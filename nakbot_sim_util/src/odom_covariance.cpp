#include <iostream>
#include <vector>
#include <random>

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

    std::random_device seed_gen;



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

        std::mt19937 engine(seed_gen());
        std::normal_distribution<double> dist_pos(0.0, 0.1);
        std::normal_distribution<double> dist_quat(0.0, 0.05);

        odom_repub.pose.pose.position.x += dist_pos(engine);
        odom_repub.pose.pose.position.y += dist_pos(engine);
        odom_repub.pose.pose.position.z += dist_pos(engine);

        odom_repub.pose.pose.orientation.x += dist_quat(engine);
        odom_repub.pose.pose.orientation.y += dist_quat(engine);
        odom_repub.pose.pose.orientation.z += dist_quat(engine);
        odom_repub.pose.pose.orientation.w += dist_quat(engine);


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