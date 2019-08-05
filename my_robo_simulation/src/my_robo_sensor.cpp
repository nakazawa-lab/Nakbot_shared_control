#include "my_robo_simulation/my_robo_drive.h"
#include "my_robo_simulation/dwa_normal.h"

#include "my_robo_simulation/my_robo_sensor.h"

/*
static void calculate_dwa(my_robo_spec spec,float predict_time, float dt, std::vector<pair_vel>& candidate_vel){

}
*/




void my_robo_sensor::cb_odom(const nav_msgs::Odometry::ConstPtr &msg)
{
    // ROS_INFO("odom vel %f", msg->twist.twist.linear.x);
    odom = *msg;

}

void my_robo_sensor::cb_lrf(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // 受け取ったメッセージをコピーしておく
    latest_scan = *msg;
    if (count == 0)
    {
        ROS_INFO("first laser scan.");
        range_point = latest_scan.ranges.size(); // 取得した点の数
        center = range_point / 2;
        ROS_INFO("range_point:%d.", range_point);
        ROS_INFO("center:%d.", center);
        count++;
    }
    // <TODO>>
    // 測定範囲外の場合の対応
}

void my_robo_sensor::cb_joy(const sensor_msgs::Joy::ConstPtr &joy_msg)
{
    // // ジョイスティック左側
    // // 上→axes[1]の正方向
    // // 左→axes[0]の正方向
    // joy_cmd_vel[0] =spec.x_max_vel*joy_msg.axes[1];
    // //cmd_vel.linear.y =joy_msg.axes[2];

    // if(joy_msg.axes[1]>=0) joy_cmd_vel[1]=spec.z_max_ang*joy_msg.axes[0];

    // else joy_cmd_vel[1] = -1*spec.z_max_ang*joy_msg.axes[0];

    // ROS_INFO("x_joy: %f",joy_cmd_vel[0]);
    // ROS_INFO("z_ang: %f\n",joy_cmd_vel[1]);

    // メッセージを保管しておく
    joy = *joy_msg;

    if (countj == 0)
    {
        ROS_INFO("first joy sub.");
        countj++;
    }
    //ROS_INFO("count j:%d",countj);
}

visualization_msgs::MarkerArray my_robo_sensor::make_obs_markers(std::vector<std::vector<double>> obs)
{
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(obs.size());

    int k = 0;
    // 候補の数/4だけループ
    for (int i = 0; i < obs.size(); i++)
    {
        //ROS_INFO("start put marker.");

        // float GREEN= (double)rand()/RAND_MAX;

        //予測時刻の数だけループ
        // for (int j = 0; j < DWA.PredictTraj[i].size(); j++)
        // {

        //ROS_INFO("start loop.");

        marker_array.markers[k].header.frame_id = "/odom";
        marker_array.markers[k].header.stamp = ros::Time::now();
        marker_array.markers[k].ns = "cmd_vel_display";
        marker_array.markers[k].id = k;
        marker_array.markers[k].lifetime = (ros::Duration)1;

        // marker_array.markers[j].type = visualization_msgs::Marker::CUBE;
        marker_array.markers[k].type = visualization_msgs::Marker::SPHERE;
        marker_array.markers[k].action = visualization_msgs::Marker::ADD;
        marker_array.markers[k].scale.x = 0.1;
        marker_array.markers[k].scale.y = 0.1;
        marker_array.markers[k].scale.z = 0.1;
        marker_array.markers[k].pose.position.x = obs[i][0];
        marker_array.markers[k].pose.position.y = obs[i][1];
        marker_array.markers[k].pose.position.z = 0;
        marker_array.markers[k].pose.orientation.x = 0;
        marker_array.markers[k].pose.orientation.y = 0;
        marker_array.markers[k].pose.orientation.z = 0;
        marker_array.markers[k].pose.orientation.w = 1;

        marker_array.markers[k].color.r = 0.0f;
        marker_array.markers[k].color.g = 0.0f;
        marker_array.markers[k].color.b = 1.0f;
        marker_array.markers[k].color.a = 1.0f;
        k++;
    }
    return marker_array;
}

void my_robo_sensor::pub_joy_marker(){
    
}