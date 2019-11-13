#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "JetSAS/ros_node.h"

// void JetSAS_Node::pub_lrf(){
//     lrf_pub.publish(*scan);
//     delete[] scan;
// }

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

void JetSAS::Lrf::make_scan_msgs(long* urg_data,const int scan_num){
    //std::cout << "in make scan msgs, urg_data num is " << scan_num <<std::endl;

    //std::cout << "urg_data[0] " << urg_data[0]/100.0 <<" " << urg_angle_increment <<" "<< scan.angle_increment<<std::endl;

    if (scan_num ==0){
        std::cout << "no scan msgs" <<std::endl;
    }
    else{
        for(int i=0; i<scan_num;i++){
            //std::cout << "urg data[" << i <<"] is " << urg_data[i] << " " << scan.angle_increment <<std::endl;
            scan.ranges[i] = urg_data[scan_num-i-1]/100.0;
        }
        //std::cout << "finish make sensor msgs" <<std::endl;
    }
}

void JetSAS::Odom::cal_now_vel(const double this_loop_time){
    assert(this_loop_time != 0);

    right_v = (encoder_right - old_encoder_right) / this_loop_time * encoder_multiplier_vel;
    left_v = (encoder_left - old_encoder_left) / this_loop_time * encoder_multiplier_vel;

    // ここの符号は実験の結果変わるかもしれない
    v = (right_v + left_v) / 2;
    w = (right_v - left_v) / robot_width; 

    old_encoder_right =  encoder_right;
    old_encoder_left = encoder_left;
}

void JetSAS::Odom::cal_pose(double dt){
    double vx = v * old_p.cos_th;
    double vy = v * old_p.sin_th;

    now_p.x = old_p.x + vx * dt;
    now_p.y = old_p.y + vy * dt;

    // cos(th+wt),sin(th+wt)を求める
    now_p.cos_th = add_theorem_cos(old_p.sin_th, sin(w * dt), old_p.cos_th, cos(w * dt));
    now_p.sin_th = add_theorem_sin(old_p.sin_th, sin(w * dt), old_p.cos_th, cos(w * dt));

    old_p = now_p;
}

void JetSAS::Odom::make_odom_msgs(int e_right, int e_left, double this_loop_time){
    set_encoder(e_right,e_left);
    cal_now_vel(this_loop_time);
    cal_pose(this_loop_time);


    // quaternion(x,y,z,w)に対して,方向ベクトル(lambda_x,lambda_y,lambda_z)に角度theta だけ回転するとき
    // x = lambda_x * sin(theta/2)
    // y = lambda_y * sin(theta/2)
    // z = lambda_z * sin(theta/2)
    // w = cos(theta/2)
    double sin_half_theta = sqrt((1-now_p.cos_th)/2);
    double cos_half_theta = sqrt((1-now_p.cos_th)/2);
    odom.pose.pose.position.x = now_p.x;
    odom.pose.pose.position.y = now_p.y;

    odom.pose.pose.orientation.x = sin_half_theta;
    odom.pose.pose.orientation.y = sin_half_theta;
    odom.pose.pose.orientation.z = sin_half_theta;
    odom.pose.pose.orientation.w = cos_half_theta;

    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;
}

void JetSAS_Node::controlloop(){
    // urgの値をとってくる　get_urg

    // エンコーダの値をサンプルプログラムからとってくる jetstas e

    // RCの値をサンプルプログラムからとってくる jetsas r

    // odom, lrf, joyをpublish
    pub_sensor();

    // subscribeした情報をshが理解できる値に変換する
    cmd_vel.cmd_vel_to_encoder();

    // SHに送信する jetsas v
}