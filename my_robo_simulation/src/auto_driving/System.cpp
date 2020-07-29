#include "auto_driving/System.hpp"

namespace JetSAS
{
System::System() : rate_(10)
{
    ROS_INFO("system contructor");

};

System::~System()
{
    ROS_INFO("system destructor");

}

void System::mainloop()
{
    ROS_INFO("system mainloop start");

    while (ros::ok())
    {
        ROS_INFO("loop");
        
        // goalが存在するか確かめる
        bool is_valid_goal = goal_ptr_->has_valid_goal();

        if (is_valid_goal)
        {
            // latest_scanが危険かどうかを判定
            bool is_dangerous = scan_ptr_->is_dangerous();

            if(is_dangerous)
            {
                // 危険な場合、進行方向を変えるために新しい速度を設計
                velocity_ptr_->update_velocity();

            }

        }
        else
        {
            // goalが存在しないときは、0速度を選ぶ
            velocity_ptr_->make_stay_vel();
        }

        // 速度をpublish
        velocity_ptr_->publish_vel();

        rate_.sleep();
    }
}

void System::set_publisher()
{

}

}