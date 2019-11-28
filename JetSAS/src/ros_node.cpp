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
bool IsFirstRes = true;         // 初回受信を確認するフラグ

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

void save_serial(const char &RS_cmd, const int (&RS_prm)[4])
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

        if(IsFirstRes){
            std::cout << "IsFirstRes" << std::endl;
            IsFirstRes = false;
            ros_serial.encoder.r_init = RS_prm[2];
            ros_serial.encoder.l_init = RS_prm[3];
        }
    }
    else
    {
        std::cout << "RS_cmd except r or e" << std::endl;
    }

    //std::cout << "result " << ros_serial.rc.rot << " " << ros_serial.encoder.r_ref << " " <<
    //ros_serial.rc.lin << " " << ros_serial.encoder.l_ref << " "<< std::endl; 
}

void JetSAS_Node::register_vel_param(){
    //nh.setParam("/my_robo/diff_drive_controller/angular/z/has_acceleration_limits",true);
/*
 * /my_robo/diff_drive_controller/angular/z/has_acceleration_limits: True
 * /my_robo/diff_drive_controller/angular/z/has_velocity_limits: True
 * /my_robo/diff_drive_controller/angular/z/max_acceleration: 1.5
 * /my_robo/diff_drive_controller/angular/z/max_velocity: 1.5
 * /my_robo/diff_drive_controller/angular/z/min_acceleration: -1.5
 * /my_robo/diff_drive_controller/angular/z/min_velocity: -1.5
 * /my_robo/diff_drive_controller/base_frame_id: base_footprint
 * /my_robo/diff_drive_controller/cmd_vel_timeout: 10.0
 * /my_robo/diff_drive_controller/left_wheel: left_wheel_joint
 * /my_robo/diff_drive_controller/linear/x/has_acceleration_limits: True
 * /my_robo/diff_drive_controller/linear/x/has_velocity_limits: True
 * /my_robo/diff_drive_controller/linear/x/max_acceleration: 1.0
 * /my_robo/diff_drive_controller/linear/x/max_velocity: 0.8
 * /my_robo/diff_drive_controller/linear/x/min_acceleration: -1.0
 * /my_robo/diff_drive_controller/linear/x/min_velocity: -0.8
 * /my_robo/diff_drive_controller/odom_frame_id: odom
 * /my_robo/diff_drive_controller/pose_covariance_diagonal: [0.001, 0.001, 10...
 * /my_robo/diff_drive_controller/publish_rate: 50.0
 * /my_robo/diff_drive_controller/right_wheel: right_wheel_joint
 * /my_robo/diff_drive_controller/twist_covariance_diagonal: [0.001, 0.001, 10...
 * /my_robo/diff_drive_controller/type: diff_drive_contro...
 * /my_robo/diff_drive_controller/wheel_radius: 0.075
 * /my_robo/diff_drive_controller/wheel_separation: 0.285
*/
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

    // right_v = (encoder_right - old_encoder_right) / this_loop_time * encoder_multiplier;
    // left_v = (encoder_left - old_encoder_left) / this_loop_time * encoder_multiplier;

    // std::cout << "in cal now vel " << encoder_right << " " << old_encoder_right <<std::endl;
    // std::cout << "in cal now vel " << encoder_left << " " << old_encoder_left <<std::endl;
    // std::cout << right_v << " " << left_v << " " <<std::endl;
    // std::cout << "this loop time " <<this_loop_time <<std::endl;
    // // ここの符号は実験の結果変わるかもしれない
    // v = (right_v + left_v) / 2.0;
    // w = (right_v - left_v) / robot_width;

    // std::cout << "from position encoder, (v,w) is " << v  << ", " << w << std::endl;
    // std::cout << "right v left v " <<right_v << " " << left_v  <<std::endl;

    right_v = (ros_serial.encoder.r_ref - INTERCEPT_ENCODER) * encoder_multiplier / this_loop_time;
    left_v = (ros_serial.encoder.l_ref - INTERCEPT_ENCODER) * encoder_multiplier / this_loop_time;
    
    std::cout << ros_serial.encoder.r_ref << " " << ros_serial.encoder.r_ref - INTERCEPT_ENCODER << " " << encoder_multiplier << std::endl;
    std::cout << "v[m/s] from enc " << right_v << " " << left_v << std::endl; 
    v = (right_v + left_v) / 2.0;
    w = (right_v - left_v) / robot_width;

    std::cout << "from vel encoder, (v,w) is " << v  << ", " << w << std::endl;
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

    //std::cout << "(x, y, sin_th, cos_th) " << now_p.x << ", " << now_p.y << ", "<< now_p.sin_th << ", "<< now_p.cos_th <<std::endl;
}

//     geometry_msgs::Twist vel;        を
//     int encoder_prm1, encoder_prm2;　に変換する
void JetSAS::Cmd_vel::cmd_vel_to_encoder(){
    double tmp = vel.angular.z * robot_width / 2.0;
    encoder_cmd_r = (vel.linear.x + tmp) * cmd_multiplier_to_enc + INTERCEPT_ENCODER;
    encoder_cmd_l = (vel.linear.x - tmp) * cmd_multiplier_to_enc + INTERCEPT_ENCODER;

    std::cout << "in cmd_vel encoder_prm_r and encoder_prm_l " << encoder_cmd_r << " " << encoder_cmd_l << std::endl; 
}

void JetSAS::RC::rc_to_encoder(){
    v_right_enc = ros_serial.rc.lin *rc_multiplier_vel_r + vel_r_int + ros_serial.rc.rot * rc_multiplier_rot_r + rot_r_int + INTERCEPT_ENCODER;
    v_left_enc = ros_serial.rc.lin *rc_multiplier_vel_l + vel_l_int + ros_serial.rc.rot * rc_multiplier_rot_l + rot_l_int + INTERCEPT_ENCODER;

    std::cout << "rc to encoder" << v_right_enc << " " << v_left_enc <<std::endl;
}

void JetSAS::Joy::make_joy_msgs(){
    float center_lin =  (max_rc_lin + min_rc_lin)/ 2.0;
    float center_rot = (max_rc_rot + min_rc_rot)/ 2.0;

    joy_axes1 = (ros_serial.rc.lin - center_lin) / (max_rc_lin - min_rc_lin);
    joy_axes0 = (ros_serial.rc.rot - center_rot) / (max_rc_rot - min_rc_rot);
    //std::cout << "in make joy msgs " << joy_axes1 << " " << joy_axes0 << std::endl;

    joy.axes[0] = joy_axes0;
    joy.axes[1] = joy_axes1;
}

bool JetSAS::Odom::check_new_encoder(){
    bool a = (ros_serial.encoder.r_sum == encoder_right);
    bool b = (ros_serial.encoder.l_sum == encoder_left);
    bool c = (ros_serial.encoder.r_ref == encoder_right_ref);
    bool d = (ros_serial.encoder.l_ref == encoder_right_ref);
    
    if(a && b && c && d) return false;
    else return true;
}

bool JetSAS::RC::check_new_rc(){
    if(ros_serial.rc.rot == rc_rot) return false;
    else return true;
}


void JetSAS_Node::controlloop(JET_TIMER &jt){
    int n;
    long time_stamp;
    //std::cout << "start control function" << std::endl;

    // エンコーダの値をサンプルプログラムからとってくる jetstas e
    jetsas('e',0001,0001);
    // RCの値をサンプルプログラムからとってくる jetsas r
    jetsas('r',0001,0001);

    // 値が変化したときだけ実行する
    if(odom.check_new_encoder() && rc.check_new_rc()){
        rc.set_rc(ros_serial.rc.rot);

        // urgの値をとってくる
        urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
        n = urg_get_distance(&urg, urg_data, &time_stamp);
        if (n < 0)
        {
            printf("urg_get_distance: %s\n", urg_error(&urg));
            urg_close(&urg);
        }
        lrf.make_scan_msgs(urg_data,n);

        // エンコーダの値をもとに現在の位置と速度を計算する
        auto dur = std::chrono::system_clock::now() - last_cal_time;
        cal_time = (double)std::chrono::duration_cast<std::chrono::milliseconds>(dur).count()/1000.0;
        std::cout << "cal time " << cal_time << std::endl;
        odom.make_odom_msgs(ros_serial.encoder.r_sum, ros_serial.encoder.l_sum,cal_time);

        last_cal_time = std::chrono::system_clock::now();

        // RCの値をもとに現在の人間からの速度指令値を計算する
        joy.make_joy_msgs();

        // odom, lrf, joyをpublish
        pub_sensor();

        // 提案手法に基づき計算された/cmd_velトピックをsubscribeした情報をshが理解できる値に変換する
        //cmd_vel.cmd_vel_to_encoder();

        // SHに送信する jetsas v
        // jetsas('v',encoder_prm_r,encoder_prm_l);
        std::cout << std::endl;
        write_log();
    }
    
}

void JetSAS_Node::make_log_col(){
    // パラメータを入れる拡張あり

    logfile << std::endl; 
    // 列を入れる
    std::string col_name = "time,e1_right_vel,e2_left_vel,e3_right_sum,e4_left_sum,r1_rot,r2_lin,r3_chan3,r4_chan4";
    logfile << col_name << std::endl;
}

void JetSAS_Node::write_log(){
  auto dur = std::chrono::system_clock::now() - start_time;
  auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();

    logfile << (double)(msec/1000.0) << "," << ros_serial.encoder.r_ref << "," << ros_serial.encoder.l_ref << "," << 
ros_serial.encoder.r_sum << "," << 
ros_serial.encoder.l_sum << "," << ros_serial.rc.rot << "," << ros_serial.rc.lin << "," << ros_serial.rc.chan3 << "," << 
ros_serial.rc.chan4 <<std::endl;
}
