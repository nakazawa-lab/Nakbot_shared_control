#include <iomanip>
#include <urg_c/urg_sensor.h>
#include <urg_c/urg_utils.h>

#include "JetSAS/SimpleGPIO.h"
#include "JetSAS/jetsas.h"
#include "JetSAS/open_urg_sensor.h"

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include "JetSAS/ros_node.h"
#include "tf/transform_datatypes.h"

extern long *urg_data;
extern urg_t urg;
extern int jetsas(char, int, int);
bool IsFirstRes = true;
std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> last_res_time_r, last_res_time_e;
std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> res_start_time;
std::vector<double> e_res_time;
std::vector<double> r_res_time;

void flush_res_time(){
    std::ofstream res_time_file;
    res_time_file.open("./log_JetSAS/restime_log_"+get_current_time()+".csv");

    res_time_file << "e res time,r res time" << std::endl;

    for(int i = 0; i<e_res_time.size();i++){
        res_time_file 
        << e_res_time[i] << ',' 
        << r_res_time[i] << ','
        << std::endl;
    }
    res_time_file.close();
}

JetSAS_Node::~JetSAS_Node(){
    std::cout << "JetSAS destructor" <<std::endl;
    logfile.close();

    flush_res_time();
};

double add_theorem_sin(double sin_a, double sin_b, double cos_a, double cos_b)
{
  double a = (sin_a * cos_b + cos_a * sin_b);
  return a;
}

double add_theorem_cos(double sin_a, double sin_b, double cos_a, double cos_b)
{
  double a = (cos_a * cos_b - sin_a * sin_b);
  return a;
}

void disp_pros_time(std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> start_time, std::string str){
    auto cal_time = (double)std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - last_res_time_r).count()/1000.0;
    std::cout << str << "  " << "time interval " << cal_time << std::endl;
    if(str == "res: e"){
        e_res_time.push_back(cal_time);
    }
    else if(str == "res: r"){
        r_res_time.push_back(cal_time);
    }
}

void save_serial(const char &RS_cmd, const int (&RS_prm)[4])
{
    if (RS_cmd == 'r')
    {
        ros_serial.rc.rot = RS_prm[0];
        ros_serial.rc.lin = RS_prm[1];
        ros_serial.rc.chan3 = RS_prm[2];
        ros_serial.rc.chan4 = RS_prm[3];

        disp_pros_time(last_res_time_r, "res: r");
        last_res_time_r = std::chrono::system_clock::now();
    }
    else if (RS_cmd == 'e')
    {
        ros_serial.encoder.r_ref = RS_prm[0];
        ros_serial.encoder.l_ref = RS_prm[1];
        ros_serial.encoder.r_sum = RS_prm[2];
        ros_serial.encoder.l_sum = RS_prm[3];

        disp_pros_time(last_res_time_e, "res: e");
        last_res_time_e = std::chrono::system_clock::now();

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

void JetSAS::Odom::set_encoder(const int e_right, const int e_left)
{
    old_encoder_right = encoder_right;
    old_encoder_left = encoder_left;

    encoder_right = e_right;
    encoder_left = e_left;

    encoder_right_ref = ros_serial.encoder.r_ref;
    encoder_left_ref = ros_serial.encoder.l_ref;
    
    //std::cout << "in set encoder " << old_encoder_right << " " << encoder_right << " " << e_right << std::endl;
}

void JetSAS::Lrf::make_scan_msgs(long* urg_data){
    scan.header.frame_id="/scan";
    if (scan_num ==0){
        std::cout << "no scan msgs" <<std::endl;
    }
    else{
        for(int i=0; i<scan_num;i++){
            //std::cout << "urg data[" << i <<"] is " << urg_data[i] << " " << scan.angle_increment <<std::endl;
            //scan.ranges[i] = urg_data[scan_num-i-1]/1000.0;
            scan.ranges[i] = urg_data[i]/1000.0;
        }
    }
}

void JetSAS::Odom::cal_now_vel(const double this_loop_time){
    assert(this_loop_time != 0);
    if(!IsFirstRes){
        // right_v = (encoder_right - old_encoder_right) / this_loop_time * rc_to_encoder;
        // left_v = (encoder_left - old_encoder_left) / this_loop_time * rc_to_encoder;

        // //std::cout << "in cal now vel " << encoder_right << " " << old_encoder_right <<std::endl;
        // //std::cout << "in cal now vel " << encoder_left << " " << old_encoder_left <<std::endl;
        // std::cout << right_v << " " << left_v << " " <<std::endl;
        // //std::cout << "this loop time " <<this_loop_time <<std::endl;
        // // ここの符号は実験の結果変わるかもしれない
        // v = (right_v + left_v) / 2.0;
        // w = (right_v - left_v) / (robot_width);

        // //std::cout << "from position encoder, (v,w) is " << v  << ", " << w << std::endl;
        // std::cout << "from position encoder,right v left v " <<right_v << " " << left_v  <<std::endl;

        right_v = (ros_serial.encoder.r_ref - INTERCEPT_ENCODER) * enc_to_vel;
        left_v = (ros_serial.encoder.l_ref - INTERCEPT_ENCODER) * enc_to_vel;

        //std::cout << ros_serial.encoder.r_ref << " " << ros_serial.encoder.l_ref - INTERCEPT_ENCODER << " " << enc_to_vel << std::endl;
        //std::cout << "v[m/s] from enc " << right_v << " " << left_v << std::endl; 
        v = (right_v + left_v) / 2.0;
        w = (right_v - left_v) / (robot_width);

        //std::cout << "from vel encoder, (v,w) is " << v  << ", " << w << std::endl;
    }
}

void JetSAS::Odom::cal_pose(double dt){
    double vx = v * old_p.cos_th;
    double vy = v * old_p.sin_th;

    now_p.x = old_p.x + vx * dt;
    now_p.y = old_p.y + vy * dt;
    now_p.cos_th = add_theorem_cos(old_p.sin_th, sin(w * dt), old_p.cos_th, cos(w * dt));
    now_p.sin_th = add_theorem_sin(old_p.sin_th, sin(w * dt), old_p.cos_th, cos(w * dt));
    now_p.th = old_p.th + w*dt;

    old_p = now_p;

    //std::cout << "now p: (" << now_p.x << ", " << now_p.y << ")" << "cos,sin: " << now_p.cos_th << " " << now_p.sin_th << std::endl;
}

void JetSAS::Odom::make_odom_msgs(const int e_right, const int e_left,const double this_loop_time){
    set_encoder(e_right,e_left);
    cal_now_vel(this_loop_time);
    cal_pose(this_loop_time);


    // quaternion(x,y,z,w)に対して,方向ベクトル(lambda_x,lambda_y,lambda_z)に角度theta だけ回転するとき
    // x = lambda_x * sin(theta/2)
    // y = lambda_y * sin(theta/2)
    // z = lambda_z * sin(theta/2)
    // w = cos(theta/2)
    // double theta;
    // if(now_p.sin_th>=0)theta = acos(now_p.cos_th);
    // else theta = acos(now_p.cos_th) + M_PI;
    tf::Quaternion quaternion=tf::createQuaternionFromRPY(/*roll=*/0,/*pitch=*/0,/*yaw=*/now_p.th);
    geometry_msgs::Quaternion quat_Msg;
    quaternionTFToMsg(quaternion,quat_Msg);//この関数はROSのライブラリ

    odom.header.frame_id = "/odom";
    odom.header.stamp = ros::Time::now();
    odom.pose.pose.position.x = now_p.x;
    odom.pose.pose.position.y = now_p.y;
    odom.pose.pose.orientation = quat_Msg;
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;

    // std::cout << "pose " <<std::endl;
    // std::cout << odom.pose.pose.position.x <<" " << odom.pose.pose.position.y << " " << now_p.th << std::endl;
    // std::cout << odom.twist.twist.linear.x <<" " << odom.twist.twist.angular.z << std::endl; 
    // //std::cout << "(x, y, sin_th, cos_th) " << now_p.x << ", " << now_p.y << ", "<< now_p.sin_th << ", "<< now_p.cos_th <<std::endl;
}

//     geometry_msgs::Twist vel;        を
//     jetsas(e)で得られる形式, 及びjetsas(v)で送信する形式　に変換する
void JetSAS::Cmd_vel::cmd_vel_to_jetsas_prm(){
    double tmp = vel.angular.z * robot_width/2.0;
    double encoder_cmd_r = ((vel.linear.x + tmp) * cmd_multiplier_to_enc) + INTERCEPT_ENCODER;
    double encoder_cmd_l = ((vel.linear.x - tmp) * cmd_multiplier_to_enc) + INTERCEPT_ENCODER;

    jetsas_e_r = (int)(encoder_cmd_r - INTERCEPT_ENCODER)*10 +5000;
    jetsas_e_l = (int)(encoder_cmd_l - INTERCEPT_ENCODER)*10 +5000;

    // std::cout << "in cmd_vel (v,w)=(" << vel.linear.x << "," << vel.angular.z << "),(right, left)="
    //  << vel.linear.x +tmp<< "," << vel.angular.z -tmp<< ")"<<std::endl;
    // std::cout << "in cmd_vel encoder_prm_r and encoder_prm_l " << std::setprecision(7)<< encoder_cmd_r << " " << encoder_cmd_l << std::endl; 
    // std::cout << "in cmd_vel jetsas param " << jetsas_e_r << " " << jetsas_e_l << std::endl; 

}

void JetSAS::RC::rc_to_encoder(){
    v_right_enc = ros_serial.rc.lin *rc_multiplier_vel_r + vel_r_int + ros_serial.rc.rot * rc_multiplier_rot_r + rot_r_int + INTERCEPT_ENCODER;
    v_left_enc = ros_serial.rc.lin *rc_multiplier_vel_l + vel_l_int + ros_serial.rc.rot * rc_multiplier_rot_l + rot_l_int + INTERCEPT_ENCODER;

    //std::cout << "rc to encoder" << v_right_enc << " " << v_left_enc <<std::endl;
}

void JetSAS::Joy::make_joy_msgs(){
    joy_axes1 = -2.0*(ros_serial.rc.lin - center_lin) / (max_rc_lin - min_rc_lin);
    joy_axes0 = -2.0*(ros_serial.rc.rot - center_rot) / (max_rc_rot - min_rc_rot);

    joy.header.frame_id = "/joy";
    joy.header.stamp = ros::Time::now();
    joy.axes[0] = joy_axes0;
    joy.axes[1] = joy_axes1;
    
    //std::cout << "in make joy msgs " << joy_axes1 << " " << joy_axes0 << std::endl;
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
    if(ros_serial.rc.rot == rc_rot_) return false;
    else return true;
}

void JetSAS_Node::res_urg()
{
  int i;
  long time_stamp;
//   while(1){
    // urgの値をとってくる
    urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
    lrf.scan_num = urg_get_distance(&urg, urg_data, &time_stamp);
    if (lrf.scan_num < 0)
    {
        printf("urg_get_distance: %s\n", urg_error(&urg));
        urg_close(&urg);
    }
  //}
}


void JetSAS_Node::controlloop(JET_TIMER &jt){
    
    auto start = std::chrono::system_clock::now();
    res_urg();

    lrf.make_scan_msgs(urg_data);

    // エンコーダの値をもとに現在の位置と速度を計算する
    auto dur = std::chrono::system_clock::now() - last_cal_time;
    last_cal_time = std::chrono::system_clock::now();
    cal_time = (double)std::chrono::duration_cast<std::chrono::milliseconds>(dur).count()/1000.0;
    //std::cout << "cal time " << cal_time << std::endl;
    odom.make_odom_msgs(ros_serial.encoder.r_sum, ros_serial.encoder.l_sum,cal_time);

    // RCの値をもとに現在の人間からの速度指令値を計算する
    rc.set_rc(ros_serial.rc.rot);
    joy.make_joy_msgs();

    // odom, lrf, joyをpublish
    pub_sensor();

    // 提案手法に基づき計算された/cmd_velトピックをsubscribeした情報をshが理解できる値に変換する
    cmd_vel.cmd_vel_to_jetsas_prm(); 
}

void JetSAS_Node::make_log_col(){
    // パラメータを入れる拡張あり

    logfile << std::endl; 
    std::string col_name 
    = "time,e1_right_vel,e2_left_vel,e3_right_sum,e4_left_sum,r1_rot,r2_lin,r3_chan3,r4_chan4,pos_x,pos_y,pos_th,joy_lin,joy_rot,modi_lin,modi_rot";
    logfile << col_name << std::endl;
}

void JetSAS::Lrf::make_scan_log_col(){
    // パラメータを入れる拡張あり

    scanlogfile << std::endl; 
    scanlogfile << "time,index,range" << std::endl;
}

void JetSAS::Lrf::write_scan_log(const double sec){
    for(int i =0; i<scan_num;i++){
        scanlogfile
        << sec << ","
        << i << ","
        << scan.ranges[i]
        << std::endl;
    }
}


void JetSAS_Node::write_log(){
  auto dur = std::chrono::system_clock::now() - start_time;
  auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
  double sec = (double)(msec/1000.0);

    logfile 
    << sec << "," 
    << ros_serial.encoder.r_ref << "," 
    << ros_serial.encoder.l_ref << "," 
    << ros_serial.encoder.r_sum << "," 
    << ros_serial.encoder.l_sum << "," 
    << ros_serial.rc.rot << "," 
    << ros_serial.rc.lin << "," 
    << ros_serial.rc.chan3 << "," 
    << ros_serial.rc.chan4 << ","
    << odom.odom.pose.pose.position.x << ","
    << odom.odom.pose.pose.position.y << ","
    << odom.now_p.th << ","
    << joy.joy.axes[1] << ","
    << joy.joy.axes[0] << ","
    << cmd_vel.vel.linear.x << ","
    << cmd_vel.vel.angular.z << ","
    <<std::endl;

    if(lrf.scan_num!=0)lrf.write_scan_log(sec);
}
