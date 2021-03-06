#include "my_robo_simulation/my_robo_drive.h"
#include <vector>
#include <algorithm>
#include <ctime>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <experimental/filesystem>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "my_robo_simulation/my_robo_util.h"
#include "my_robo_simulation/MyDWA.h"

FILE *gp; // gnuplotに指令を与えるためのテキストファイル

//#define PABLODWA
#define MYDWA
#define ISSHARED
//#define PUB_MARKER
//#define REAL_TEST

// 何かキーが押されたときにループを抜けるための関数
int kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
}

std::string get_current_time()
{
    time_t timer;     /* 時刻を取り出すための型（実際はunsigned long型） */
    struct tm *local; /* tm構造体（時刻を扱う */

    /* 年月日と時分秒保存用 */
    int year, month, day, hour, minute, second;

    timer = time(NULL);        /* 現在時刻を取得 */
    local = localtime(&timer); /* 地方時に変換 */

    /* 年月日と時分秒をtm構造体の各パラメタから変数に代入 */
    year = local->tm_year + 1900; /* 1900年からの年数が取得されるため */
    month = local->tm_mon + 1;    /* 0を1月としているため */
    day = local->tm_mday;
    hour = local->tm_hour;
    minute = local->tm_min;
    second = local->tm_sec;

    return std::to_string(month) + std::to_string(day) + std::to_string(hour) + std::to_string(minute);
}

void trans_inf(sensor_msgs::LaserScan &scan)
{
    for (int i = 0; i < scan.ranges.size(); i++)
    {
        if (isinf(scan.ranges[i]))
        {
            // std::cout << "inf: " << ((scan.angle_increment * i) - (scan.angle_increment * scan.ranges.size() / 2))  * RAD2DEG << std::endl;
            scan.ranges[i] = 100;
        }
    }
};

my_robo::my_robo()
{
    spec.get_spec_param(n, spec);
    sensor.count = 0;
    sensor.countj = 0;

    // 最後の引数はメッセージを受け取ったときに呼び出す関数がクラスの中にある際に、その【実体】を指定する
    sub_odom = n.subscribe("/odom", 10, &my_robo_sensor::cb_odom, &(this->sensor));
    sub_lrf = n.subscribe("/scan", 10, &my_robo_sensor::cb_lrf, &(this->sensor));
    sub_joy = n.subscribe("/joy", 10, &my_robo_sensor::cb_joy, &(this->sensor));
    pub_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    pub_mark = n.advertise<visualization_msgs::Marker>("/marker", 1);

    pub_mark_arr = n.advertise<visualization_msgs::MarkerArray>("/marker_array", 1);

    // pubする速度指令の初期化
    vel.linear.x = 0;
    vel.angular.z = 0;

    // joyの初期化
    sensor.joy_cmd_vel[0] = 0;
    sensor.joy_cmd_vel[1] = 0;

    spec.set_resolution(spec.x_max_acc * dt / DWA_RESOLUTION_DIV, spec.z_max_acc * dt / DWA_RESOLUTION_DIV);
}

// スペック上の最大加速度と今の速度からDynamicWindowを求める vector<vector<float>>型のCanVelに格納される
// CandVel_v[i] = v [i][1]=w
void my_robo::cal_DWA()
{
    // 現在の速度(odonm)から刻み時間後に到達可能な最大、最小速度を求める。それを一時保存しておく
    double max_dwa_vel = sensor.odom.twist.twist.linear.x + spec.x_max_acc * dt;
    double min_dwa_vel = sensor.odom.twist.twist.linear.x + spec.x_min_acc * dt;
    double max_dwa_ang = sensor.odom.twist.twist.angular.z + spec.z_max_acc * dt;
    double min_dwa_ang = sensor.odom.twist.twist.angular.z + spec.z_min_acc * dt;

    // それらがスペックを超えていたら、スペックの限度の値に保税する
    if (max_dwa_vel > spec.x_max_vel)
        max_dwa_vel = spec.x_max_vel;
    if (min_dwa_vel < spec.x_min_vel)
        min_dwa_vel = spec.x_min_vel;
    if (max_dwa_ang > spec.z_max_ang)
        max_dwa_ang = spec.z_max_ang;
    if (min_dwa_ang < spec.z_min_ang)
        min_dwa_ang = spec.z_min_ang;
    // ROS_INFO("DWA:%f,%f,%f,%f.",max_dwa_vel,max_dwa_ang,min_dwa_vel,min_dwa_ang);

    // 幅に関して、解像度だけ繰り返して組み合わせを作る
    double f = min_dwa_vel;
    double g = min_dwa_ang;
    int i = 0;

    //ROS_INFO("puch back candidates.");

    // はじめに、現在の速度を入れる
    if (sensor.odom.twist.twist.linear.x >= 0)
    {
        //CandVel.push_back(std::vector<double>());
        // CandVel[i].push_back(sensor.odom.twist.twist.linear.x);  //[i][0]に速度要素
        // CandVel[i].push_back(sensor.odom.twist.twist.angular.z); //[i][1]に角速度要素
        CandVel_v.push_back(sensor.odom.twist.twist.linear.x);  //[i][0]に速度要素
        CandVel_w.push_back(sensor.odom.twist.twist.angular.z); //[i][1]に角速度要素
        //ROS_INFO("CandVel:%f,%f",CandVel_v[i],CandVel_w[i]);
    }
    i++;

    //もし速度0がDWAに含まれていれば、それも候補に加える
    if (0 >= min_dwa_vel && 0 <= max_dwa_ang && 0 >= min_dwa_ang && 0 <= max_dwa_ang)
    {
        CandVel_v.push_back(0); //[i][0]に速度要素
        CandVel_w.push_back(0); //[i][1]に角速度要素
        //ROS_INFO("CandVel:%f,%f",CandVel_v[i],CandVel_w[i]);
        i++;
    }

    while (true)
    {
        double g = min_dwa_ang;

        // ウインドウを角速度方向に捜査
        while (true)
        {
            // 後ろ方向に進む候補に対しては考えない
            if (f >= 0)
            {
                CandVel_v.push_back(f); //[i][0]に速度要素
                CandVel_w.push_back(g); //[i][1]に角速度要素
                //ROS_INFO("CandVel:%f,%f",CandVel_v[i],CandVel_w[i]);
                i++;
            }

            g += spec.ang_res;
            if (g > max_dwa_ang)
                break;
        }
        //　速度方向に1つ増やし、次のループでまた角速度方向に捜査
        f += spec.vel_res;
        if (f > max_dwa_vel)
            break;
    }
    //ROS_INFO("candSize:%d.",CandVel.size());
}

const void my_robo::push_back_traj(const int i, const double time, const position np, const double d, double theta)
{
    // timeとpをPredictTrajに格納する処理
    PredictTraj.back().push_back(std::vector<double>());
    PredictTraj.back().back().push_back(time);
    PredictTraj.back().back().push_back(np.x);
    PredictTraj.back().back().push_back(np.y);
    PredictTraj.back().back().push_back(np.sin_th);
    PredictTraj.back().back().push_back(np.cos_th);

    //PredictTraj_r.back().push_back(std::vector<double>());

    if (CandVel_w[i] < 0)
        theta -= M_PI;

    // PredictTraj_r.back().back().push_back(time);
    // PredictTraj_r.back().back().push_back(d);
    // PredictTraj_r.back().back().push_back(theta);

    //std::cout << "PredictTraj r:(" << theta << ", " << d << ")" << std::endl;
    //std::cout << "candVel " << CandVel_v[i] << " " << CandVel_w[i] << std::endl;
}

// 予測軌道を計算する関数 内部でmy_robo.robot_modelを呼びだす
void my_robo::cal_predict_position()
{
    double time, d, theta, radius;
    ROS_INFO("candidate size is %d", CandVel_v.size());
    // すべての候補に対して
    for (int i = 0; i < CandVel_v.size(); i++)
    {
        time = dt_traj;

        position p = cal_nowp(sensor.odom);
        //ROS_INFO("x=%f,y=%f,cos_h=%f,sin_th=%f",p.x,p.y,p.cos_th,p.sin_th);

        // indexの挿入
        PredictTraj.push_back(std::vector<std::vector<double>>());
        //PredictTraj_r.push_back(std::vector<std::vector<double>>());

        position np;

        if (CandVel_w[i] == 0)
        {
            while (time <= PredictTime)
            {

                np = robot_model(p, CandVel_v[i], CandVel_w[i], dt_traj);

                //radius = 0;
                //d = CandVel_v[i] * time;
                //theta = 0;

                push_back_traj(i, time, np, d, theta);

                p = np;
                time += dt_traj;
            }
        }
        else
        {
            while (time <= PredictTime)
            {
                np = robot_model(p, CandVel_v[i], CandVel_w[i], dt_traj);

                //radius = fabs(CandVel_v[i] / CandVel_w[i]);
                //d = sqrt(2 * (1 - cos(CandVel_w[i] * time))) * radius;
                //double temp = radius * sin(CandVel_w[i] * time) / d;
                // if (temp < -1)
                // {
                //     //std::cout << "tmep is lower -1" << std::endl;
                //     temp = -1;
                // }
                // else if (temp > 1)
                // {
                //     //std::cout << "temp is upper 1" <<std::endl;
                //     temp = 1;
                // }
                // theta = acos(temp); // acosは-1から1までの値を受け取り0からpiまでの値を返す

                // if (isnan(theta))
                // {
                //     std::cout << "theta is nan:" << d << "," << radius << "," << temp << ", " << CandVel_w[i] * time << std::endl
                //               << std::endl;
                //     ;
                // }

                push_back_traj(i, time, np, d, theta);

                p = np;
                time += dt_traj;
            }
        }
    }

    //ROS_INFO("candidate size is %d",CandVel.size());
    //ROS_INFO("finish cal traj\n");

    // joyの軌道の予測
    time = dt_traj;
    position p = cal_nowp(sensor.odom);
    while (time <= PredictTime)
    {
        // 位置の更新
        position np_joy;
        np_joy = robot_model(p, sensor.joy_cmd_vel[0], sensor.joy_cmd_vel[1], dt_traj);
        //ROS_INFO("push1");
        // timeとpをJoy_PredictTrajに格納する処理
        Joy_PredictTraj.push_back(std::vector<double>());
        Joy_PredictTraj.back().push_back(time);
        Joy_PredictTraj.back().push_back(np_joy.x);
        Joy_PredictTraj.back().push_back(np_joy.y);
        Joy_PredictTraj.back().push_back(np_joy.sin_th);
        Joy_PredictTraj.back().push_back(np_joy.cos_th);
        p = np_joy;
        time += dt_traj;
    }
}

void MyDWA::DWAloop()
{
    ROS_INFO("control loop start.");

    ros::Rate rate(looprate);
    start_time = std::chrono::system_clock::now();
    bool plot_flag = false;
    int marker_loop_flag = 0; // n周期に1回trueにする
    int control_loop_flag = 0;

    record_param();

    while (ros::ok())
    {
        ros::spinOnce();

        ROS_INFO("get DWA loop.");

        loop_start_time = std::chrono::system_clock::now();

        if (sensor.latest_scan.ranges.size() == 0)
        {
            ROS_INFO("no LRF data.waiting...");
            usleep(1000);
        }
        else
        {

            //trans_inf(sensor.latest_scan);
            if ((dt == (control_loop_flag / looprate)) || (control_loop_flag == 0))
            {
                check_joy();
                ROS_INFO("ISCONTROL");
                clear_vector();
                control_loop_flag = 0;

                visualization_msgs::MarkerArray obsmarkers = sensor.make_obs_markers();
                //pub_marker_array(obsmarkers);

                cal_DWA();
                if (CandVel_v.size()==0)goto end;
                //LOG.push_back(CandVel.size());
                //say_time("cal DWA", loop_start_time);

                cal_predict_position();
                //say_time("predict position", loop_start_time);
#ifndef REAL_TEST
#ifdef PABLODWA
                //cal_Dist();
                cal_Dist2();
                say_time("cal dist", loop_start_time);

                // LOG.push_back(cal_average_d_U(CandVel));

                // double D = cal_vel_sat();
                double D = 1;

                cal_J_sharedDWA(D);
                say_time("cal J", loop_start_time);
#endif
#ifdef MYDWA
                Proposed();
                //say_time("proposed", loop_start_time);
#endif

                if (sensor.joy_cmd_vel[0] >= -0 && (sensor.odom.twist.twist.linear.x >= 0))
                {
#ifdef PUB_MARKER
                    visualization_msgs::MarkerArray markers;
                    if (++marker_loop_flag == PUB_TRAJ_MARKER_PER_LOOP)
                    {
                        // 最終的に選択した軌道のマーカ、joyのマーカーと、予測軌道のマーカを作成、表示する
                        markers = make_traj_marker_array(opt_index);
                        pub_marker_array(markers);

                        markers = make_joy_traj_marker_array();
                        pub_marker_array(markers);
                        marker_loop_flag = 0;

#ifdef MYDWA
                        position p;
                        if ((dist_lin_ang[opt_index][2] == 99999999))
                        {
                            p.x = 0;
                            p.y = 0;
                        }
                        else
                        {
                            p = sensor.index_to_pos(dist_lin_ang[opt_index][2]);
                        }
                        visualization_msgs::Marker marker = make_nearest_LRF_marker(p.x, p.y);
                        pub_marker(marker);

                        // marker = make_nearest_LRF_marker(sensor.odom.pose.pose.position.x,sensor.odom.pose.pose.position.y);
                        // pub_marker(marker);
#endif
                    }
                    say_time("pub marker", loop_start_time);
#endif
                }

                if (sensor.joy_cmd_vel[0] > -0)
                {
#ifdef ISSHARED
#ifdef PABLODWA
                    vel.linear.x = CandVel_v[opt_index];
                    vel.angular.z = CandVel_w[opt_index];
#endif
#ifdef MYDWA
                    vel.linear.x = CandVel_v[opt_index];
                    vel.angular.z = CandVel_w[opt_index];

                    ROS_INFO("opt_idx: %d, pubvel:%f,%f", opt_index, vel.linear.x, vel.angular.z);
#endif
#endif
                }
#endif      // REAL_TEST
                //sensor.print_deg_range();
                //sensor.print_odom();

                if (plot_flag)
                {
                    // <gnuplot> //
                    plot_gnuplot(gp);
                    say_time("plot", loop_start_time);
                }
            }

end:
            pub_cmd.publish(vel);
            std::cout << "pubvel (" << vel.linear.x << ", " << vel.angular.z << ")" << std::endl;
            std::cout << "joy vel:" << sensor.joy_cmd_vel[0] << "," << sensor.joy_cmd_vel[1] << std::endl;
            //say_time("pub", loop_start_time);
            cal_end_time = std::chrono::system_clock::now();
            record_loop_info();
            // say_time("record", loop_start_time);
            control_loop_flag++;
        }

        rate.sleep();
        say_time("waiting", loop_start_time);

        // なにかのキーが押されていることの判定
        if (kbhit())
        {
            int c;
            c = getchar();
            printf("'%c'を押しました。\n ループを抜けるには「e」、プロットを開始するなら「p」、プロットを終えるなら「q」\n", c);
            putchar(c);
            if (c == 'e')
                break;
            else if (c == 'p')
                plot_flag = true;
            else if (c == 'q')
                plot_flag = false;
        }
        ROS_INFO("\n");
    }
}

void MyDWA::record_loop_info()
{

    auto now = std::chrono::system_clock::now();
    auto timestanp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time).count();

    auto loop_cal_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(cal_end_time - loop_start_time).count();

    std::vector<float>::iterator minIt = std::min_element(sensor.latest_scan.ranges.begin(), sensor.latest_scan.ranges.end());
    int minScanIdx = std::distance(sensor.latest_scan.ranges.begin(), minIt);
#ifdef MYDWA
    mylogfile 
    << (double)(timestanp_ms / 1000.0) << "," 
    << sensor.odom.pose.pose.position.x << "," 
    << sensor.odom.pose.pose.position.y << "," 
    << selected.linadm << "," 
    << selected.linsafe << "," 
    << selected.angadm << "," 
    << selected.angsafe << "," 
    << selected.vel_h_cost << ","
    << selected.head_h_cost << "," 
    << selected.cost << "," 
    << selected.vel << "," 
    << selected.ang << "," 
    << sensor.joy_cmd_vel[0] << "," 
    << sensor.joy_cmd_vel[1] << "," 
    << selected.lindist << "," 
    << selected.angdist << "," 
    << sensor.odom.twist.twist.linear.x << "," 
    << sensor.odom.twist.twist.angular.z << "," 
    << loop_cal_time_ms << ","
    << *std::min_element(sensor.latest_scan.ranges.begin(), sensor.latest_scan.ranges.end()) << ","
    << sensor.index_to_rad(minScanIdx) * RAD2DEG 
    << std::endl;
#endif

#ifdef PABLODWA

    logfile 
    << (double)(timestanp_ms / 1000.0) << "," 
    << sensor.odom.pose.pose.position.x << "," 
    << sensor.odom.pose.pose.position.y << "," 
    << selected.adm << "," 
    << 1 - selected.adm << "," 
    << selected.vel_h_cost << ","
    << selected.head_h_cost << "," 
    << selected.cost << "," 
    << CandVel_v[opt_index] << "," 
    << CandVel_w[opt_index] << "," 
    << sensor.joy_cmd_vel[0] << "," 
    << sensor.joy_cmd_vel[1] << "," 
    << sensor.odom.twist.twist.linear.x << "," 
    << sensor.odom.twist.twist.angular.z << ","  
    << loop_cal_time_ms << ","
    << *std::min_element(sensor.latest_scan.ranges.begin(), sensor.latest_scan.ranges.end()) << ","
    << sensor.index_to_rad(minScanIdx) * RAD2DEG 
    << std::endl;
#endif
}

void log_init(MyDWA& robot){
    std::string date = get_current_time();
    #ifdef MYDWA
    robot.IsProposed = true;
    robot.k_heading = 2.0;
    std::string mylogfilename = "/home/kitajima/catkin_ws/src/Nakbot_shared_control/my_robo_simulation/log/mylog_" + date + ".csv";
    robot.mylogfile.open(mylogfilename);
#endif
#ifdef PABLODWA
    robot.IsProposed = false;
    robot.k_heading=1.0;
    std::string logfilename = "/home/kitajima/catkin_ws/src/Nakbot_shared_control/my_robo_simulation/log/log_" + date + ".csv";
    robot.logfile.open(logfilename);
#endif
    if(robot.IsREAL){
        std::string logfilename = "./my_robo_simulation/log/mylog_" + date + ".csv";
        robot.logfile.open(logfilename);
    }

}

void gnuplot_init(){
    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set multiplot\n");
    fprintf(gp, "set xrange [-5:5]\n");
    fprintf(gp, "set yrange [-5:5]\n");
    fprintf(gp, "set xlabel \"x[m]\"\n");
    fprintf(gp, "set ylabel \"y[m]\"\n");
}

void close_file(MyDWA& robot){
    robot.logfile.close();
    robot.mylogfile.close();
    pclose(gp);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_robo_drive");
    MyDWA robot;
    //log_init(robot);
    std::string date = get_current_time();

#ifdef MYDWA
    robot.IsProposed = true;
    robot.k_heading = 1.5;
    std::string mylogfilename = "/home/kitajima/catkin_ws/src/Nakbot_shared_control/my_robo_simulation/log/mylog_" + date + ".csv";
    robot.mylogfile.open(mylogfilename);
#endif
#ifdef PABLODWA
    robot.IsProposed = false;
    robot.k_heading=1.0;
    std::string logfilename = "/home/kitajima/catkin_ws/src/Nakbot_shared_control/my_robo_simulation/log/log_" + date + ".csv";
    robot.logfile.open(logfilename);
#endif
    if(robot.IsREAL){
        std::string logfilename = "./my_robo_simulation/log/mylog_" + date + ".csv";
        robot.logfile.open(logfilename);
    }


    //gnuplot_init();

    robot.DWAloop();

    // close_file(robot);

    return 0;
}
