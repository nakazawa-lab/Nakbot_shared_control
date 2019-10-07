#include "my_robo_simulation/my_robo_drive.h"
#include "my_robo_simulation/dwa_normal.h"

#include "my_robo_simulation/my_robo_sensor.h"
#include"my_robo_simulation/my_robo_util.h"


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
        point_num = latest_scan.ranges.size(); // 取得した点の数
        center = point_num / 2;
        ROS_INFO("point_num:%d.", point_num);
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
    for (int i = 0; i < obs.size(); i++)
    {
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
        // 直線状に並んでいると思われる2つの点を見つけたら
        if(abs(scan.ranges[i]-scan.ranges[i+1]) < lineTh){
            sameLineflag = true;
            startPoint = i;

            // 次の点も直線上にあるか調べる
            while(true){
                if(abs(scan.ranges[i]-scan.ranges[i+1]) < lineTh){
                    if(i==scan.ranges.size()-2) {
                        endPoint =i-1;
                        break;
                        }
                    else i++;
                }

                else {
                    endPoint =i;
                    break;
                }
            }
            

            // starpointとendpointが決まったので、新しいlineオブジェクトを保存
            line newLine = line(startPoint, endPoint, scan);
            lines.push_back(newLine);
            //ROS_INFO("detect new line.");
            sameLineflag=false;
        }
        i++;
    }
    ROS_INFO("num of line: %d",lines.size());
}


double line::cal_curve(int startPoint, int endPoint, const sensor_msgs::LaserScan &scan){
    double Dist_startToend;
    // double startX,startY,endX,endY;

    // メモのルーズリーフ参照.余弦定理
    double alpha, beta,start_theta,end_theta;

    start_theta = cal_ang_fromfront(startPoint, scan);
    end_theta =  cal_ang_fromfront(endPoint, scan);

    //ROS_INFO("start theta: %f, end theta: %f",start_theta, end_theta);

    //ROS_INFO("start point: %d, end point: %d",startPoint, endPoint);

    // 現在の位置から見た点の相対座標 メンバに代入
    start_X = scan.ranges[startPoint] * cos(start_theta);
    start_Y = scan.ranges[startPoint] * sin(start_theta);
    end_X = scan.ranges[endPoint] * cos(end_theta);
    end_Y = scan.ranges[endPoint] * sin(end_theta);

    Dist_startToend = sqrt( (start_X - end_X) * (start_X - end_X) + (start_Y - end_Y) * (start_Y - end_Y) );

    //ROS_INFO("dist start to end: %f",Dist_startToend);

    // 余弦定理
    beta = acos (((Dist_startToend * Dist_startToend) + (scan.ranges[endPoint] * scan.ranges[endPoint]) - (scan.ranges[startPoint] * scan.ranges[startPoint]))
                / (2 * Dist_startToend * scan.ranges[endPoint]));

    alpha = beta + scan.angle_increment * (endPoint - startPoint);
    if(alpha > M_PI/2){
        alpha = M_PI - alpha;
    }

    //ROS_INFO("alpha: %f",alpha);

    return alpha + start_theta;
}

double line::cal_ang_fromfront(int index, const sensor_msgs::LaserScan &scan){
    double theta;
    float a=scan.angle_increment;
    int b=(index - scan.ranges.size()/2);

    // 以下のように書くと変な値が出てくる
    //theta = abs(scan.angle_increment * (index - scan.ranges.size()/2));
    //ROS_INFO("scan.angle_increment * (index - scan.ranges.size()/2):%f",theta);

    theta = a*b;
    //ROS_INFO("scan.angle_increment * (index - scan.ranges.size()/2):%f",a*b);
    //ROS_INFO("scan.angle_increment * (index - scan.ranges.size()/2):%f",theta);


    return theta;
}

// 線のはじめと終わりのロボット座標系での位置を計算し、そのマーカを返す関数
visualization_msgs::MarkerArray line::make_edge_marker(int center,const sensor_msgs::LaserScan &scan, geometry_msgs::PoseWithCovariance &pose){
    std::vector<std::vector<double>> lineedge;

    double roll, yaw, pitch;
    // 今のrpyを求める
    geometry_quat_to_rpy(roll, pitch, yaw, pose.pose.orientation);

    /// ----------------start point--------------------- ///
    lineedge.push_back(std::vector<double>());
    // ロボット座標から見たときの座標xs,ys 絶対座標xo,yo
    double xs = scan.ranges[start_point_index] * cos((start_point_index-center) * scan.angle_increment);
    double ys = scan.ranges[start_point_index] * sin((start_point_index-center) * scan.angle_increment);
    double xo = pose.pose.position.x + cos(yaw) * xs - sin(yaw) * ys;
    double yo = pose.pose.position.y + sin(yaw) * xs + cos(yaw) * ys;
    lineedge.back().push_back(xo);
    lineedge.back().push_back(yo);

    /// ------------------------end point--------------------------- ///
    lineedge.push_back(std::vector<double>());
    // ロボット座標から見たときの座標xs,ys 絶対座標xo,yo
    xs = scan.ranges[start_point_index] * cos((start_point_index-center) * scan.angle_increment);
    ys = scan.ranges[start_point_index] * sin((start_point_index-center) * scan.angle_increment);
    xo = pose.pose.position.x + cos(yaw) * xs - sin(yaw) * ys;
    yo = pose.pose.position.y + sin(yaw) * xs + cos(yaw) * ys;

    lineedge.back().push_back(xo);
    lineedge.back().push_back(yo);



    return make_markers_2Dvector(lineedge);
}

// scanを受け取ってthごと、左右wthまでの点の座標を計算し、そのマーカーの位置を計算する関数を呼び出して返す
// 計算される障害物の座標は絶対座標
visualization_msgs::MarkerArray my_robo_sensor::cal_obs(sensor_msgs::LaserScan &scan, double th, double wth, geometry_msgs::PoseWithCovariance &pose)
{
    obs.clear();
    // th 度相当の点の数degpを求める
    int thp = (int)((th * DEG2RAD) / latest_scan.angle_increment);
    // ROS_INFO("degp:%d",thp);
    // 10度ずつ70度までなら7倍
    int deg_inc = (int)(wth / th);

    double roll, yaw, pitch;
    // 今のrpyを求める
    geometry_quat_to_rpy(roll, pitch, yaw, pose.pose.orientation);

    //ROS_INFO("deg_inc:%d",deg_inc);
    for (int i = -deg_inc; i < deg_inc; i++)
    {
        if (scan.ranges[center + thp * i] < scan.range_max && scan.ranges[center + thp * i] > scan.range_min)
        {
            obs.push_back(std::vector<double>());
            // ロボット座標から見たときの座標xs,ys 絶対座標xo,yo
            double xs = scan.ranges[center + thp * i] * cos(th * DEG2RAD * i);
            double ys = scan.ranges[center + thp * i] * sin(th * DEG2RAD * i);
            double xo = pose.pose.position.x + cos(yaw) * xs - sin(yaw) * ys;
            double yo = pose.pose.position.y + sin(yaw) * xs + cos(yaw) * ys;

            obs.back().push_back(xo);
            obs.back().push_back(yo);
            // ROS_INFO("obs x:%f, y:%f",xo,yo);
        }
    }
    return make_obs_markers(obs);
}

double my_robo_sensor::index_to_rad(int index){
    return latest_scan.angle_increment * (index - center);
}


// インデックスから絶対座標を取得
position my_robo_sensor::index_to_pos(int scanId){
    position p;
    if(isnan(scanId) || isinf(scanId)){
        p.x=0;
        p.y=0;    
    }
    else{
        double roll, yaw, pitch;
        // 今のrpyを求める
        geometry_quat_to_rpy(roll, pitch, yaw, odom.pose.pose.orientation);

        // ロボット座標から見たときの座標xs,ys 絶対座標xo,yo
        double xs = latest_scan.ranges[scanId] * cos(index_to_rad(scanId));
        double ys = latest_scan.ranges[scanId] * sin(index_to_rad(scanId));
        double xo = odom.pose.pose.position.x + cos(yaw) * xs - sin(yaw) * ys;
        double yo = odom.pose.pose.position.y + sin(yaw) * xs + cos(yaw) * ys;

        p.x = xo;
        p.y = yo;
    }

    return p;
}