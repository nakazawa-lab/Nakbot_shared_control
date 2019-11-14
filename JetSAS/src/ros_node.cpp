#include <urg_c/urg_sensor.h>
#include <urg_c/urg_utils.h>

#include "JetSAS/SimpleGPIO.h"
#include "JetSAS/jetsas.h"
#include "JetSAS/open_urg_sensor.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "JetSAS/ros_node.h"

extern long *urg_data;
extern urg_t urg;
extern int jetsas(char, int, int);

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

void save_serial(char &RS_cmd, int (&RS_prm)[4])
{
    if (RS_cmd == 'r')
    {
        //std::cout << "in save serial RS_cmd r" << std::endl;
        ros_serial.rc.rot = RS_prm[0];
        ros_serial.rc.lin = RS_prm[1];
        ros_serial.rc.chan3 = RS_prm[2];
        ros_serial.rc.chan4 = RS_prm[3];
    }
    else if (RS_cmd == 'e')
    {
        //std::cout << "in save serial RS_cmd e" << std::endl;
        ros_serial.encoder.r_ref = RS_prm[0];
        ros_serial.encoder.l_ref = RS_prm[1];
        ros_serial.encoder.r_sum = RS_prm[2];
        ros_serial.encoder.l_sum = RS_prm[3];
    }
    else
    {
        std::cout << "RS_cmd except r or e" << std::endl;
    }

    //std::cout << "result " << ros_serial.rc.rot << " " << ros_serial.encoder.r_ref << " " <<
    //ros_serial.rc.lin << " " << ros_serial.encoder.l_ref << " "<< std::endl; 
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

    //std::cout << "in cal now vel " << encoder_right << " " << old_encoder_right <<std::endl;
    // ここの符号は実験の結果変わるかもしれない
    v = (right_v + left_v) / 2.0;
    w = (right_v - left_v) / robot_width;

    //std::cout << "from position encoder, (v,w) is " << v  << ", " << w << std::endl;
    //std::cout << "right v left v " <<right_v << " " << left_v  <<std::endl;

    right_v_from_enc = (ros_serial.encoder.r_ref - 5000) * encoder_multiplier_vel;
    left_v_from_enc = (ros_serial.encoder.l_ref - 5000) * encoder_multiplier_vel;
    
    //std::cout << "right v from enc " << right_v_from_enc << " " << left_v_from_enc << std::endl; 
    v_from_enc = (right_v_from_enc + left_v_from_enc) / 2.0;
    w_from_enc = (right_v_from_enc - left_v_from_enc) / robot_width;

    //std::cout << "from vel encoder, (v,w) is " << v_from_enc  << ", " << w_from_enc << std::endl;

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

void JetSAS::Odom::make_odom_msgs(const int e_right, const int e_left,const double this_loop_time){
    //std::cout << "in make_odom_msgs" << e_right << " " << e_left << std::endl;
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

    std::cout << "(x, y, sin_th, cos_th) " << now_p.x << ", " << now_p.y << ", "<< now_p.sin_th << ", "<< now_p.cos_th <<std::endl;
}

void JetSAS::Cmd_vel::cmd_vel_to_encoder(){
    //     geometry_msgs::Twist vel;        を
    //     int encoder_prm1, encoder_prm2;　に変換する
    double tmp = vel.angular.z * robot_width / 2.0;
    encoder_prm_r = vel.linear.x + tmp;
    encoder_prm_l = vel.linear.x - tmp;

    std::cout << "in cmd_vel encoder_prm_r and encoder_prm_l " << encoder_prm_r << " " << encoder_prm_l << std::endl; 
}

void JetSAS::RC::rc_to_vel(){
    // この基準1000の値を検討する必要がある
    v_h = (ros_serial.rc.lin - 1000.0) * rc_multiplier_vel;
    w_h = (ros_serial.rc.rot - 1000.0) * rc_multiplier_rot;
}

void JetSAS::Joy::make_joy_msgs(){
    // とりあえずここに書くがクラスのメンバ変数にする
    float max_rc_lin=1200.0, max_rc_rot=1200.0, min_rc_lin=1170.0, min_rc_rot=1170.0;
    float center_lin =  (max_rc_lin + min_rc_lin)/ 2.0;
    float center_rot = (max_rc_rot + min_rc_rot)/ 2.0;

    joy_axes1 = (ros_serial.rc.lin - center_lin) / (max_rc_lin - min_rc_lin);
    joy_axes0 = (ros_serial.rc.rot - center_rot) / (max_rc_rot - min_rc_rot);
    std::cout << "in make joy msgs " << joy_axes1 << " " << joy_axes0 << std::endl;

    joy.axes[0] = joy_axes0;
    joy.axes[1] = joy_axes1;
}



void JetSAS_Node::controlloop(JET_TIMER &jt){
    int n;
    long time_stamp;
    std::cout << "start control function" << std::endl;

    // urgの値をとってくる　get_urg
    urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
    n = urg_get_distance(&urg, urg_data, &time_stamp);
    if (n < 0)
    {
        printf("urg_get_distance: %s\n", urg_error(&urg));
        urg_close(&urg);
    }
    lrf.make_scan_msgs(urg_data,n);

    // エンコーダの値をサンプルプログラムからとってくる jetstas e
    jetsas('e',0001,0001);

    // エンコーダの値をもとに現在の位置と速度を計算する
    this_loop_time = (double)(jt.get_nsec() / 1000000000.0) - old_time;
    //std::cout <<"this loop time " << this_loop_time << std::endl;
    odom.make_odom_msgs(ros_serial.encoder.r_sum, ros_serial.encoder.l_sum,this_loop_time);

    // RCの値をサンプルプログラムからとってくる jetsas r
    jetsas('r',0001,0001);

    // RCの値をもとに現在の人間からの速度指令値を計算する
    //rc_to_vel();
    joy.make_joy_msgs();

    // odom, lrf, joyをpublish
    pub_sensor();

    // 提案手法に基づき計算された/cmd_velトピックをsubscribeした情報をshが理解できる値に変換する
    //cmd_vel.cmd_vel_to_encoder();

    // SHに送信する jetsas v
    // jetsas('v',hoge,fuga);
    std::cout << std::endl;
}
