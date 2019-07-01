#include"my_robo_simulation/my_robo_drive.h"

/*
void my_robo::cb_lrf(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // 受け取ったメッセージをコピーしておく
    latest_scan = *msg;

    // <TODO>>
    // 測定範囲外の場合の対応
}
*/
//sensor_msgs/*::Header*/& msg

//int count=0;
/*
void chatterCallback(const sensor_msgs::LaserScan& msg){
    if(count==0){
        ROS_INFO("I heard angle_min: [%f]", static_cast<float>(msg.angle_min));
        ROS_INFO("I heard angle_max: [%f]", static_cast<float>(msg.angle_max));
        ROS_INFO("I heard angle_increment: [%f]", static_cast<float>(msg.angle_increment));
        ROS_INFO("I heard scan_time: [%f]", static_cast<float>(msg.scan_time));
        ROS_INFO("I heard range_min: [%f]", static_cast<float>(msg.range_min));
        ROS_INFO("I heard range_max: [%f]", static_cast<float>(msg.range_max));
    count++;
    }

  ROS_INFO("I heard: [%f]", static_cast<float>(msg.ranges[msg.ranges.size()/2]));
    //ROS_INFO("I heard: [%f]", static_cast<float>(msg.intensities[0]));    強度は測定不可
}*/
/*
int main(int argc, char **argv){
  ros::init(argc, argv, "Laser_listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/laserscan", 10, chatterCallback);
  ros::Rate loop_rate(1);

  ros::spin();
  return 0;
}
*/
/*
void chatterCallback(const std_msgs::String& msg){
  ROS_INFO("I heard: [%s]", msg.data.c_str());
}

int main(int argc, char **argv){c
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 10, chatterCallback);
  ros::Rate loop_rate(10);

  ros::spin();
  return 0;
}
*/
