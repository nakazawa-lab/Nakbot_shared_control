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

void my_robo::clear_vector(){
  // ループの最後にはpredicttrajectoryやcmdcandidateなどを消去する
  DWA.CandVel.clear();
  DWA.PredictTraj.clear();
  DWA.isCollision.clear();
  // ROS_INFO("candsize:%d",DWA.CandVel.size());
  // ROS_INFO("predict:%d",DWA.PredictTraj.size());
  // ROS_INFO("isCollisiton:%d",DWA.isCollision.size());
}

void my_robo::check_joy(){

  if (sensor.countj != 0)
  {
    // joyからの速度司令の計算
    // sensor.joy_cmd_vel[0] 速度
    // sensor.joy_cmd_vel[1] 角速度

    // ジョイスティック左側
    // 上→axes[1]の正方向
    // 左→axes[0]の正方向
    sensor.joy_cmd_vel[0] = spec.x_max_vel * sensor.joy.axes[1];
    //cmd_vel.linear.y =joy_msg.axes[2];

    if (sensor.joy.axes[1] >= 0)
      sensor.joy_cmd_vel[1] = spec.z_max_ang * sensor.joy.axes[0];
    else
      sensor.joy_cmd_vel[1] = -1 * spec.z_max_ang * sensor.joy.axes[0];

    // ROS_INFO("spec.x: %f",spec.x_max_vel);
    // ROS_INFO("spec.z: %f\n",spec.z_max_ang);

    ROS_INFO("x_joy: %f", sensor.joy_cmd_vel[0]);
    ROS_INFO("z_ang: %f\n", sensor.joy_cmd_vel[1]);

    vel.linear.x = sensor.joy_cmd_vel[0];
    vel.angular.z = sensor.joy_cmd_vel[1];
  }
}
