#include"my_robo_simulation/my_robo_drive.h"
#include"visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include<random>

#include<my_robo_simulation/my_robo_sensor.h>
#include"my_robo_simulation/my_robo_util.h"



void my_robo::clear_vector(){
  // ループの最後にはpredicttrajectoryやcmdcandidateなどを消去する
  DWA.CandVel.clear();
  DWA.PredictTraj.clear();
  DWA.isCollision.clear();
  DWA.Joy_PredictTraj.clear();
  sensor.lines.clear();
  // ROS_INFO("candsize:%d",DWA.CandVel.size());
  // ROS_INFO("predict:%d",DWA.PredictTraj.size());
  // ROS_INFO("isCollisiton:%d",DWA.isCollision.size());
}

double cal_average_d_U(std::vector<std::vector<double>>& CandVel){
  int size = CandVel.size();
  double sum=0;

  for(int i= 0; i< size; i++){
    sum += CandVel[i][2];
  }

  return sum / size;
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
visualization_msgs::MarkerArray my_robo::make_traj_marker_array(int index)
{
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize((DWA.PredictTraj[0].size()+1) * DWA.PredictTraj.size());

  int k=0;
  float green=0;
  float red=0;
  bool flag = false;  // 採用軌道を示すもの  
// 候補の数ループ
  for (int i=0; i<DWA.PredictTraj.size();i ++){
    //ROS_INFO("start put marker.");

    // float GREEN= (double)rand()/RAND_MAX;
    if(i == index)flag = true;
    else flag = false;

    //予測時刻の数だけループ
    for (int j = 0; j < DWA.PredictTraj[i].size(); j +=2)
    {
 
       //ROS_INFO("start loop.");

      marker_array.markers[k].header.frame_id = "/odom";
      marker_array.markers[k].header.stamp = ros::Time::now();
      marker_array.markers[k].ns = "cmd_vel_display";
      marker_array.markers[k].id = k;
      marker_array.markers[k].lifetime = (ros::Duration)(1 /DWA.looprate);   //1ループ存在

      // marker_array.markers[j].type = visualization_msgs::Marker::CUBE;
      marker_array.markers[k].type = visualization_msgs::Marker::SPHERE;
      marker_array.markers[k].action = visualization_msgs::Marker::ADD;


      if(flag){
      marker_array.markers[k].scale.x = 0.1;
      marker_array.markers[k].scale.y = 0.1;
      marker_array.markers[k].scale.z = 0.1;
      }
      else
      {
      marker_array.markers[k].scale.x = 0.05;
      marker_array.markers[k].scale.y = 0.05;
      marker_array.markers[k].scale.z = 0.05;
      }

      marker_array.markers[k].pose.position.x = DWA.PredictTraj[i][j][1];
      marker_array.markers[k].pose.position.y = DWA.PredictTraj[i][j][2];
      marker_array.markers[k].pose.position.z =  0;
      marker_array.markers[k].pose.orientation.x = 0;
      marker_array.markers[k].pose.orientation.y = 0;
      marker_array.markers[k].pose.orientation.z = 0;
      marker_array.markers[k].pose.orientation.w = 1;

      
      if(flag){
        marker_array.markers[k].color.r = 0.0f;
        marker_array.markers[k].color.g = 0.0f;
        marker_array.markers[k].color.b = 1.0f;
        marker_array.markers[k].color.a = 1.0f;
      }
      else
      {
        marker_array.markers[k].color.r = 1.0f;
        marker_array.markers[k].color.g = DWA.CandVel[i][2];
        marker_array.markers[k].color.b = DWA.CandVel[i][2];
        marker_array.markers[k].color.a = 1.0f;
      }

      k++;

    }
      // green +=0.05;
      // if(green>1)green=0;

      // red +=0.05;
      // if(red>1)red=0;
  }
 // pub_marker_array(marker_array);
  //ROS_INFO("pub marker array.");

// joyの予測軌道を緑色で入れる
   //予測時刻の数だけループ

    for (int j = 0; j < DWA.Joy_PredictTraj.size(); j ++)
    {
    //ROS_INFO("start loop.");

    marker_array.markers[k].header.frame_id = "/odom";
    marker_array.markers[k].header.stamp = ros::Time::now();
    marker_array.markers[k].ns = "cmd_vel_display";
    marker_array.markers[k].id = k;
    marker_array.markers[k].lifetime = (ros::Duration)(1 / DWA.looprate) * 2; //2ループ存在

    // marker_array.markers[j].type = visualization_msgs::Marker::CUBE;
    marker_array.markers[k].type = visualization_msgs::Marker::SPHERE;
    marker_array.markers[k].action = visualization_msgs::Marker::ADD;
    marker_array.markers[k].scale.x = 0.1;
    marker_array.markers[k].scale.y = 0.1;
    marker_array.markers[k].scale.z = 0.1;
    marker_array.markers[k].pose.position.x = DWA.Joy_PredictTraj[j][1];
    marker_array.markers[k].pose.position.y = DWA.Joy_PredictTraj[j][2];
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

void my_robo::say_log(){
  //ROS_INFO();
}

double cal_euclid(double x0, double y0, double x1, double y1)
{
    return sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
}

double add_theorem_sin(double sin_a, double sin_b, double cos_a, double cos_b)
{
    //ROS_INFO("sin_a:%f,sin_b:%f,cos_a:%f,cos_b%f",sin_a,sin_b,cos_a,cos_b);
    double a = (sin_a * cos_b + cos_a * sin_b);
    return a;
}

double add_theorem_cos(double sin_a, double sin_b, double cos_a, double cos_b)
{
    double a = (cos_a * cos_b - sin_a * sin_b);
    return a;
}

void geometry_quat_to_rpy(double &roll, double &pitch, double &yaw, geometry_msgs::Quaternion geometry_quat)
{
    tf::Quaternion quat;
    quaternionMsgToTF(geometry_quat, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw); //rpy are Pass by Reference
}

// 次の時刻のロボットの位置を計算する関数
position my_robo::robot_model(position p_now, double cand_v, double cand_w, double dt)
{
    position p_next;

    double vx = cand_v * p_now.cos_th;
    double vy = cand_v * p_now.sin_th;

    p_next.x = p_now.x + vx * dt;
    p_next.y = p_now.y + vy * dt;

    // cos(th+wt),sin(th+wt)を求める
    p_next.cos_th = add_theorem_cos(p_now.sin_th, sin(cand_w * dt), p_now.cos_th, cos(cand_w * dt));
    p_next.sin_th = add_theorem_sin(p_now.sin_th, sin(cand_w * dt), p_now.cos_th, cos(cand_w * dt));

    //ROS_INFO("sin:%f,cos:%f",p_next.sin_th,p_next.cos_th);
    //ROS_INFO("sin:%f,cos:%f,traj: x=%f,y=%f",p_next.sin_th,p_next.cos_th,p_next.x, p_next.y);
    return p_next;
}

visualization_msgs::MarkerArray make_markers_2Dvector(std::vector<std::vector<double>> obs)
{
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.resize(obs.size());
    ROS_INFO("obs size:%d",obs.size());

    int k = 0;
    for (int i = 0; i < obs.size(); i++)
    {
        marker_array.markers[k].header.frame_id = "/odom";
        marker_array.markers[k].header.stamp = ros::Time::now();
        marker_array.markers[k].ns = "cmd_vel_display";
        marker_array.markers[k].id = k;
        marker_array.markers[k].lifetime = (ros::Duration)0.25;

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

// センサが正しく動いているかのテスト用の実行ループ関数 DWAとは関係ない
void my_robo::controlloop()
{
    ROS_INFO("control loop start.");

    ros::Rate rate(5);

    // // センサデータを処理するための変数群
    int range_num, center_point, point_70;

    const double DegToPoint = 1024 / 360;
    int deg10point = 10 * DegToPoint;

    int flag = 0;

    while (ros::ok())
    {
        ros::spinOnce();
        ROS_INFO("get loop.");

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

            //ROS_INFO("x_joy: %f", sensor.joy_cmd_vel[0]);
            //ROS_INFO("z_ang: %f\n", sensor.joy_cmd_vel[1]);

            vel.linear.x = sensor.joy_cmd_vel[0];
            vel.angular.z = sensor.joy_cmd_vel[1];
        }

        // LaserScanメッセージをすでに受け取っている場合
        if (sensor.latest_scan.ranges.size() > 0)
        {

            // lrfデータに関する初期化
            if (flag == 0)
            {
                range_num = sensor.latest_scan.ranges.size();                           // 取得した点の数
                center_point = range_num / 2;                                           // 正面のスキャン点番号
                point_70 = (M_PI / (2 * sensor.latest_scan.angle_increment)) * (7 / 9); //80度となる点の数
                sensor.count++;
                ROS_INFO("lrf initialize commplete.");
                ROS_INFO("center_point:%d", center_point);
                flag++;
            }

            ROS_INFO("distance:%f\n", sensor.latest_scan.ranges[center_point]);

            // もし、正面方向に0.8m以下の距離に障害物があり、joyから前に進む司令があったとき、joyの司令を0.6倍する
            // もし、正面方向に0.4m以下の距離に障害物があり、joyから前に進む司令があったとき、joyの司令を無視して停止する

            for (int i = center_point - deg10point * 6; i <= center_point + deg10point * 6; i += deg10point)
            {
                if (sensor.latest_scan.ranges[i] < 0.8 && sensor.latest_scan.ranges[i] > 0.4 && sensor.joy_cmd_vel[0] > 0)
                {
                    ROS_INFO("close.");
                    vel.linear.x *= 0.6;
                    break;
                }
                else if (sensor.latest_scan.ranges[i] <= 0.4 && sensor.joy_cmd_vel[0] > 0)
                {
                    ROS_INFO("too close.stopped.");
                    ROS_INFO("range:%f", sensor.latest_scan.ranges[i]);
                    // ROS_INFO("angle:%f",);
                    vel.linear.x = 0.0;
                    break;
                }
            }

            //ROS_INFO("I heard: [%f]", static_cast<float>(latest_scan.ranges[range_num/2]));
            pub_cmd.publish(vel);
            ROS_INFO("pub vel.");
        }

        rate.sleep();
    }

    // ループの最後にはpredicttrajectoryやcmdcandidateなどを消去する
}