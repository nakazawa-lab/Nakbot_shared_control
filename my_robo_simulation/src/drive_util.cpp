#include "my_robo_simulation/my_robo_drive.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <random>

#include <my_robo_simulation/my_robo_sensor.h>
#include "my_robo_simulation/my_robo_util.h"
#include "my_robo_simulation/DWA_var.h"
#include "my_robo_simulation/my_robo_drive.h"
#include "my_robo_simulation/MyDWA.h"

double cal_average_d_U(std::vector<std::vector<double>> &CandVel)
{
  int size = CandVel.size();
  double sum = 0;

  for (int i = 0; i < size; i++)
  {
    sum += CandVel[i][2];
  }

  return sum / size;
}

void my_robo::check_joy()
{

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

visualization_msgs::Marker my_robo::make_pos_marker(position p)
{
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
  marker.pose.position.x = p.x;
  marker.pose.position.y = p.y;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0f;
  // pub_mark.publish(marker);
  return marker;
}

// k番目の速度候補
visualization_msgs::MarkerArray MyDWA::make_traj_marker_array(int index)
{
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize((PredictTraj[0].size() + 1) * PredictTraj.size());

  int k = 0;
  float green = 0;
  float red = 0;
  bool adopt_flag = false; // 採用軌道を示すもの

  double max_lin, max_ang, tmp = 0;
  // dist_lin_ang[i][0]と[i][1]の最大値を取得する
  for (int i = 0; i < dist_lin_ang.size(); i++)
  {
    if (tmp < (dist_lin_ang[i][0] + dist_lin_ang[i][1]))
    {
      max_lin = dist_lin_ang[i][0];
      max_ang = dist_lin_ang[i][1];
      tmp = dist_lin_ang[i][0] + dist_lin_ang[i][1];
    }
  }
  std::cout << "max dist lin ang:" << max_lin << " " << max_ang << std::endl;

  // 候補の数ループ
  for (int i = 0; i < PredictTraj.size(); i += 2)
  {
    
    //ROS_INFO("start put marker.");
    if (i == index)
      adopt_flag = true;
    else
      adopt_flag = false;
    //if((i == 0) || adopt_flag){
    //予測時刻の数だけループ
    for (int j = 0; j < PredictTraj[i].size(); j += 4)
    {
      //ROS_INFO("start loop.");
      marker_array.markers[k].header.frame_id = "/odom";
      marker_array.markers[k].header.stamp = ros::Time::now();
      marker_array.markers[k].ns = "cmd_vel_display";
      marker_array.markers[k].id = k;
      marker_array.markers[k].lifetime = (ros::Duration)( PUB_TRAJ_MARKER_PER_LOOP / looprate) ; //1ループ存在

      // marker_array.markers[j].type = visualization_msgs::Marker::CUBE;
      marker_array.markers[k].type = visualization_msgs::Marker::SPHERE;
      marker_array.markers[k].action = visualization_msgs::Marker::ADD;

      marker_array.markers[k].pose.position.x = PredictTraj[i][j][1];
      marker_array.markers[k].pose.position.y = PredictTraj[i][j][2];
      marker_array.markers[k].pose.position.z = 0;
      marker_array.markers[k].pose.orientation.x = 0;
      marker_array.markers[k].pose.orientation.y = 0;
      marker_array.markers[k].pose.orientation.z = 0;
      marker_array.markers[k].pose.orientation.w = 1;

      if (adopt_flag)
      {
        marker_array.markers[k].scale.x = 0.1;
        marker_array.markers[k].scale.y = 0.1;
        marker_array.markers[k].scale.z = 0.1;

        marker_array.markers[k].color.r = 0.0f;
        marker_array.markers[k].color.g = 0.0f;
        marker_array.markers[k].color.b = 1.0f;
        marker_array.markers[k].color.a = 1.0f;
      }
      else
      {
        double color;
        if(IsProposed){
          color = (dist_lin_ang[i][0] + dist_lin_ang[i][1]) / (max_lin + max_ang);
        }
        else{
          color = CandVel[i][2];
        }
        // double x = 1;
        // if (isCollision[i])
        // {
        //   x = 0;
        // }

        marker_array.markers[k].scale.x = 0.05;
        marker_array.markers[k].scale.y = 0.05;
        marker_array.markers[k].scale.z = 0.05;

        marker_array.markers[k].color.r = 1.0f;
        marker_array.markers[k].color.g = color;
        marker_array.markers[k].color.b = color;
        marker_array.markers[k].color.a = 1.0f;
      }
      k++;
    }
 // }
  }
  return marker_array;
}

visualization_msgs::MarkerArray MyDWA::make_joy_traj_marker_array(){
  visualization_msgs::MarkerArray marker_array;
  marker_array.markers.resize((Joy_PredictTraj[0].size() + 1) * Joy_PredictTraj.size());

  int k = 0;

  for (int j = 0; j < Joy_PredictTraj.size(); j += 3)
  {
    //ROS_INFO("start loop.");

    marker_array.markers[k].header.frame_id = "/odom";
    marker_array.markers[k].header.stamp = ros::Time::now();
    marker_array.markers[k].ns = "cmd_vel_display";
    marker_array.markers[k].id = k;
    marker_array.markers[k].lifetime = (ros::Duration)(PUB_TRAJ_MARKER_PER_LOOP / looprate);

    // marker_array.markers[j].type = visualization_msgs::Marker::CUBE;
    marker_array.markers[k].type = visualization_msgs::Marker::SPHERE;
    marker_array.markers[k].action = visualization_msgs::Marker::ADD;
    marker_array.markers[k].scale.x = 0.1;
    marker_array.markers[k].scale.y = 0.1;
    marker_array.markers[k].scale.z = 0.1;
    marker_array.markers[k].pose.position.x = Joy_PredictTraj[j][1];
    marker_array.markers[k].pose.position.y = Joy_PredictTraj[j][2];
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

position my_robo::cal_nowp(nav_msgs::Odometry &odom)
{
  position now_p;
  now_p.x = odom.pose.pose.position.x;
  now_p.y = odom.pose.pose.position.y;

  // quaternion(x,y,z,w)に対して,方向ベクトル(lambda_x,lambda_y,lambda_z)に角度theta だけ回転するとき
  // x = lambda_x * sin(theta/2)
  // y = lambda_y * sin(theta/2)
  // z = lambda_z * sin(theta/2)
  // w = cos(theta/2)
  // sin,cosの加法定理で求まる
  now_p.sin_th = 2 * odom.pose.pose.orientation.z * odom.pose.pose.orientation.w;
  now_p.cos_th = odom.pose.pose.orientation.w * odom.pose.pose.orientation.w - odom.pose.pose.orientation.z * odom.pose.pose.orientation.z;

  return now_p;
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

// 次の時刻のロボットの絶対位置を計算する関数
// p_nowが絶対位置
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
  ROS_INFO("obs size:%d", obs.size());

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

void say_time(const char *name, std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> basetime)
{
  auto temp = std::chrono::system_clock::now();
  auto dur = temp - basetime;
  auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
  ROS_INFO("after %s : %d millisec", name, msec);
}

// void my_robo::plot_d_deg_gnuplot(FILE *gp)
// {
//   // ファイルを書き出したいとき
//   // fprintf(gp,"plot \"d_theta_file.dat\" with points pointsize 10\n");

//   fprintf(gp, "clear\n");

//   // そのまま書き出したいとき
//   // fprintf(gp,"set key title \"-\"");
//   // fprintf(gp,"set key at 170,7");
//   fprintf(gp, "plot \"-\" with points pointtype 7 pointsize 0.5 lc rgb \"blue\" title \"trajectories\"\n");
//   // 候補軌道の数に対する繰り返し
//   for (int i = 0; i < PredictTraj_r.size(); i++)
//   {
//     // 軌道内の各時刻に対する繰り返し
//     for (int j = 0; j < PredictTraj_r[i].size(); j++)
//     {
//       fprintf(gp, "%f\t%f\n", PredictTraj_r[i][j][2] * RAD2DEG, PredictTraj_r[i][j][1]);
//     }
//   }
//   fprintf(gp, "e\n");
//   fflush(gp);
// }

// void my_robo::plot_d_deg_scan_gnuplot(FILE *gp)
// {
//   fprintf(gp, "plot \"-\" with points pointtype 7 pointsize 0.5 lc rgb \"red\" title \"scan\"\n");

//   for (int i = 0; i < sensor.latest_scan.ranges.size(); i++)
//   {
//     fprintf(gp, "%f\t%f\n", sensor.index_to_rad(i) * 0.53, sensor.latest_scan.ranges[i]);
//   }
//   fprintf(gp, "e\n");
//   fflush(gp);
// }

void MyDWA::plot_gnuplot(FILE *gp)
{
  fprintf(gp, "clear\n");

#pragma region 予測起動の描画
  fprintf(gp, "plot \"-\" with points pointtype 7 pointsize 0.5 lc rgb \"blue\" title \"trajectories\" \n");
  // 候補軌道の数に対する繰り返し
  for (int i = 0; i < PredictTraj_r.size(); i += 2)
  {
    // 軌道内の各時刻に対する繰り返し
    for (int j = 0; j < PredictTraj_r[i].size(); j += 2)
    {
      fprintf(gp, "%f\t%f\n", PredictTraj_r[i][j][2], PredictTraj_r[i][j][1]);
    }
  }
  fprintf(gp, "e\n");
  fflush(gp);
#pragma endregion

#pragma region スキャン点の描画
  fprintf(gp, "plot \"-\" with points pointtype 7 pointsize 0.5 lc rgb \"red\" title \"scan\"\n");
  for (int i = 0; i < sensor.latest_scan.ranges.size(); i += 3)
  {
    fprintf(gp, "%f\t%f\n", sensor.index_to_rad(i), sensor.latest_scan.ranges[i]);
  }
  fprintf(gp, "e\n");
  fflush(gp);
#pragma endregion

#pragma region ロボットの大きさの描画
  // fprintf(gp, "plot \"-\" with points pointtype 7 pointsize 0.5 lc rgb \"green\" title \"robot\"\n");
  // for (int i = 0; i < spec.RobotSize.size(); i++)
  // {
  //   fprintf(gp, "%f\t%f\n", spec.RobotSize[i][1] * RAD2DEG, spec.RobotSize[i][0]);
  // }
  // fprintf(gp, "e\n");
  // fflush(gp);
#pragma endregion
}
