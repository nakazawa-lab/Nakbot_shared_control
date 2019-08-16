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

// 線を検出する
void my_robo_sensor::detect_line(const sensor_msgs::LaserScan &scan){
    // 隣接する転換の距離がこの値を超えると同一直線状にあると判断する
    double lineTh = 0.1;
    bool sameLineflag = false;
    int startPoint;
    int endPoint;
    int i=0;

    while(i < scan.ranges.size() - 1){
        if(abs(scan.ranges[i]-scan.ranges[i+1]) < lineTh){
            sameLineflag = true;
            startPoint = i;
            i++;

            // 次の点も直線上にあるか調べる
            while(true){
                if(abs(scan.ranges[i]-scan.ranges[i+1]) < lineTh){
                    if(i==scan.ranges.size()) break;
                    else i++;
                }
                else break;
            }
            endPoint =i;

            // starpointとendpointが決まったので、新しいlineオブジェクトを保存
            line newLine = line(startPoint, endPoint);
            lines.push_back(newLine);
            ROS_INFO("detect new line.");
        }

    }
    ROS_INFO("num of line: %d",lines.size());

}

double line::cal_curve(int startPoint, int endPoint, const sensor_msgs::LaserScan &scan){
    double Dist_startToend;
    double startX,startY,endX,endY;

    // メモのルーズリーフ参照.余弦定理
    double alpha, beta,start_theta,end_theta;

    start_theta = cal_ang_fromfront(startPoint, scan);
    end_theta =  cal_ang_fromfront(endPoint, scan);

    ROS_INFO("start theta: %f, end theta: %f",start_theta, end_theta);

    // 現在の位置から見た店の相対座標
    startX = scan.ranges[startPoint] * cos(start_theta);
    startY = scan.ranges[startPoint] * sin(start_theta);
    endX = scan.ranges[endPoint] * cos(end_theta);
    endY = scan.ranges[endPoint] * sin(end_theta);

    Dist_startToend = sqrt( (startX - endX) * (startX - endX) + (startY - endY) * (startY - endY) );

    ROS_INFO("dist start to end: %f",Dist_startToend);

    // 余弦定理
    beta = acos (((Dist_startToend * Dist_startToend) + (scan.ranges[endPoint] * scan.ranges[endPoint]) - (scan.ranges[startPoint] * scan.ranges[startPoint]))
                / (2 * Dist_startToend * scan.ranges[endPoint]));

    alpha = beta + scan.angle_increment * (endPoint - startPoint);
    if(alpha > M_PI/2){
        alpha = M_PI - alpha;
    }

    ROS_INFO("alpha: %f",alpha);

    return alpha + start_theta;
}

double line::cal_ang_fromfront(int index, const sensor_msgs::LaserScan &scan){
    double theta;
    theta = abs(scan.angle_increment * (index - scan.ranges.size()/2));
    ROS_INFO("theta: %f",theta);

    return theta;
}