#include"my_robo_simulation/my_robo_drive.h"
#include"visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include<random>

#include<my_robo_simulation/my_robo_sensor.h>

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
    ROS_INFO("z_joy: %f\n", sensor.joy_cmd_vel[1]);

    vel.linear.x = sensor.joy_cmd_vel[0];
    vel.angular.z = sensor.joy_cmd_vel[1];
  }
}

visualization_msgs::Marker my_robo::make_pos_marker(position p){
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = (ros::Duration)0.5;

    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.pose.position.x=p.x;
    marker.pose.position.y=p.y;
    marker.pose.position.z=0;
    marker.pose.orientation.x=0;
    marker.pose.orientation.y=0;
    marker.pose.orientation.z=0;
    marker.pose.orientation.w=1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    // pub_mark.publish(marker);
    return marker;
}

// k番目の速度候補
visualization_msgs::MarkerArray my_robo::make_traj_marker_array()
{
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize(DWA.PredictTraj[0].size() * DWA.PredictTraj.size());

  int k=0;
  float green=0;
  float red=0;
// 候補の数/4だけループ
  for (int i=0; i<DWA.PredictTraj.size();i ++){
    //ROS_INFO("start put marker.");

    // float GREEN= (double)rand()/RAND_MAX;

    //予測時刻の数だけループ
    for (int j = 0; j < DWA.PredictTraj[i].size(); j++)
    {
 
       //ROS_INFO("start loop.");

      marker_array.markers[k].header.frame_id = "/odom";
      marker_array.markers[k].header.stamp = ros::Time::now();
      marker_array.markers[k].ns = "cmd_vel_display";
      marker_array.markers[k].id = k;
      marker_array.markers[k].lifetime = (ros::Duration)DWA.looprate;

      // marker_array.markers[j].type = visualization_msgs::Marker::CUBE;
      marker_array.markers[k].type = visualization_msgs::Marker::SPHERE;
      marker_array.markers[k].action = visualization_msgs::Marker::ADD;
      marker_array.markers[k].scale.x = 0.05;
      marker_array.markers[k].scale.y = 0.05;
      marker_array.markers[k].scale.z = 0.05;
      marker_array.markers[k].pose.position.x = DWA.PredictTraj[i][j][1];
      marker_array.markers[k].pose.position.y = DWA.PredictTraj[i][j][2];
      marker_array.markers[k].pose.position.z = 0;
      marker_array.markers[k].pose.orientation.x = 0;
      marker_array.markers[k].pose.orientation.y = 0;
      marker_array.markers[k].pose.orientation.z = 0;
      marker_array.markers[k].pose.orientation.w = 1;

      marker_array.markers[k].color.r = 0.0f;
      marker_array.markers[k].color.g = 1.0f;
      marker_array.markers[k].color.b = 0.0f;
      marker_array.markers[k].color.a = 1.0f;
      k++;

    }
      // green +=0.05;
      // if(green>1)green=0;

      // red +=0.05;
      // if(red>1)red=0;
  }
 // pub_marker_array(marker_array);
  ROS_INFO("pub marker array.");

  return marker_array;

}

position my_robo::cal_nowp(nav_msgs::Odometry& odom){
  position now_p;
  now_p.x = odom.pose.pose.position.x;
  now_p.y = odom.pose.pose.position.y;

  now_p.sin_th = 2 * odom.pose.pose.orientation.z * odom.pose.pose.orientation.w;
  now_p.cos_th = odom.pose.pose.orientation.w * odom.pose.pose.orientation.w - odom.pose.pose.orientation.z * odom.pose.pose.orientation.z;

  return now_p;
}