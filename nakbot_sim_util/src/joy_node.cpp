#include <iostream>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h" 

namespace nakbot_sim_util
{
struct my_robo_spec
{
    // ロボットの最大速度,加速度
    double x_max_vel;
    double z_max_ang;

    double x_min_vel;
    double z_min_ang;

    double x_max_acc;
    double x_min_acc;

    double z_max_acc;
    double z_min_acc;
};

class DirectJoy
{
private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher pub_vel_;
    ros::Subscriber sub_joy_;

    sensor_msgs::Joy joy_;
    my_robo_spec spec_;
    geometry_msgs::Twist vel_;

    float loop_rate_;
    bool res_first_joy_=false;
    double joy_cmd_vel_[2];      // [0]が速度[1]が角速度


    void get_spec_param();
public:
    DirectJoy(const int loop_rate=30.0)
    : loop_rate_(loop_rate)
    {
        ROS_INFO("DirectJoy");
        get_spec_param();
        sub_joy_ = nh_.subscribe("/joy", 10, &DirectJoy::joy_callback, this);
        pub_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    }
    void joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg);
    void make_vel();
    void mainloop();

};


 // スペックをパラメータから取得する関数
void DirectJoy::get_spec_param()
{
    // ロボットの最大速度,最小速度を示すパラメータを読み込む
    nh_.getParam("/my_robo/diff_drive_controller/linear/x/max_velocity", spec_.x_max_vel);
    nh_.getParam("/my_robo/diff_drive_controller/angular/z/max_velocity", spec_.z_max_ang);
    nh_.getParam("/my_robo/diff_drive_controller/linear/x/min_velocity", spec_.x_min_vel);
    nh_.getParam("/my_robo/diff_drive_controller/angular/z/min_velocity", spec_.z_min_ang);

    // ロボットの最大加速度を示すパラメータを読み込む
    nh_.getParam("/my_robo/diff_drive_controller/linear/x/max_acceleration", spec_.x_max_acc);
    nh_.getParam("/my_robo/diff_drive_controller/angular/z/max_acceleration", spec_.z_max_acc);
    nh_.getParam("/my_robo/diff_drive_controller/linear/x/min_acceleration", spec_.x_min_acc);
    nh_.getParam("/my_robo/diff_drive_controller/angular/z/min_acceleration", spec_.z_min_acc);
}

void DirectJoy::joy_callback(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
    // メッセージを保管しておく
    joy_ = *joy_msg;
    if(!res_first_joy_) res_first_joy_=true;

}

void DirectJoy::make_vel()
{
    // ジョイスティック左側
    // 上→axes[1]の正方向
    // 左→axes[0]の正方向
    joy_cmd_vel_[0] = spec_.x_max_vel * joy_.axes[1];
    //cmd_vel.linear.y =joy_msg.axes[2];

    if (joy_.axes[1] >= 0)
      joy_cmd_vel_[1] = spec_.z_max_ang * joy_.axes[0];
    else
      joy_cmd_vel_[1] = -1 * spec_.z_max_ang * joy_.axes[0];

    // ROS_INFO("spec_.x: %f",spec_.x_max_vel);
    // ROS_INFO("spec_.z: %f\n",spec_.z_max_ang);

    ROS_INFO("x_joy: %f", joy_cmd_vel_[0]);
    ROS_INFO("z_joy: %f\n", joy_cmd_vel_[1]);

    vel_.linear.x = joy_cmd_vel_[0];
    vel_.angular.z = joy_cmd_vel_[1];
}

void DirectJoy::mainloop()
{
    ros::Rate rate(loop_rate_);
    int cnt=0;

    while(ros::ok())
    {
        ROS_INFO("loop %d", cnt);   
        ros::spinOnce();

        if(!res_first_joy_)
        {
            if(cnt%10==0)
                ROS_INFO("waiting joy_...");
        }
        else
        {
            make_vel();
            pub_vel_.publish(vel_);
        }

        rate.sleep();        
        cnt++;
    }
}



} // namespace nakbot_sim_util

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_node");
    nakbot_sim_util::DirectJoy joy;
    joy.mainloop();

    return EXIT_SUCCESS;
}