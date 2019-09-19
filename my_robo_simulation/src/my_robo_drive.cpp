#include "my_robo_simulation/my_robo_drive.h"
#include <vector>
#include <algorithm>
#include <ctime>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "my_robo_simulation/my_robo_util.h"
#include "my_robo_simulation/MyDWA.h"


//std::ofstream log_traj_file;

//std::vector<double> LOG_Traj;
// LOGの中身

FILE *gp;       // gnuplotに指令を与えるためのテキストファイル

//define PABLODWA
#define MYDWA
#define ISSHARED

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

    spec.set_resolution(spec.x_max_acc * dt / 4, spec.z_max_acc * dt / 4);
}

// スペック上の最大加速度と今の速度からDynamicWindowを求める vector<vector<float>>型のCanVelに格納される
// CandVel[i][0] = v [i][1]=w
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
    CandVel.push_back(std::vector<double>());
    CandVel[i].push_back(sensor.odom.twist.twist.linear.x);  //[i][0]に速度要素
    CandVel[i].push_back(sensor.odom.twist.twist.angular.z); //[i][1]に角速度要素
    //ROS_INFO("CandVel:%f,%f",CandVel[i][0],CandVel[i][1]);
    i++;

    //もし速度0がDWAに含まれていれば、それも候補に加える
    if (0 >= min_dwa_vel && 0 <= max_dwa_ang && 0 >= min_dwa_ang && 0 <= max_dwa_ang)
    {
        CandVel.push_back(std::vector<double>());
        CandVel[i].push_back(0); //[i][0]に速度要素
        CandVel[i].push_back(0); //[i][1]に角速度要素
        //ROS_INFO("CandVel:%f,%f",CandVel[i][0],CandVel[i][1]);
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
                CandVel.push_back(std::vector<double>());
                CandVel[i].push_back(f); //[i][0]に速度要素
                CandVel[i].push_back(g); //[i][1]に角速度要素
                i++;
            }
            //ROS_INFO("CandVel:%f,%f",CandVel[i][0],CandVel[i][1]);

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

// 予測軌道を計算する関数 内部でmy_robo.robot_modelを呼びだす
void my_robo::cal_predict_position()
{
    //ROS_INFO("candidate size is %d", CandVel.size());
    // すべての候補に対して
    for (int i = 0; i < CandVel.size(); i++)
    {
        //ROS_INFO("loop %d / %d", i, CandVel.size());
        //初期化
        double time = dt_traj;

        double d,theta;
        double radius = abs(CandVel[i][0] / CandVel[i][1]);

        position p = cal_nowp(sensor.odom);
        //ROS_INFO("x=%f,y=%f,cos_h=%f,sin_th=%f",p.x,p.y,p.cos_th,p.sin_th);

        // indexの挿入
        PredictTraj.push_back(std::vector<std::vector<double>>());
        PredictTraj_r.push_back(std::vector<std::vector<double>>());


        while (time <= PredictTime)
        {
            // 位置の更新 npは絶対座標系を示す。
            // no_rは相対座標
            position np;
            np = robot_model(p, CandVel[i][0], CandVel[i][1], dt_traj);

            // timeとpをPredictTrajに格納する処理
            PredictTraj.back().push_back(std::vector<double>());
            PredictTraj.back().back().push_back(time);
            PredictTraj.back().back().push_back(np.x);
            PredictTraj.back().back().push_back(np.y);
            PredictTraj.back().back().push_back(np.sin_th);
            PredictTraj.back().back().push_back(np.cos_th);

            PredictTraj_r.back().push_back(std::vector<double>());

            d = sqrt(2 *(1 - cos(CandVel[i][1] * time))) * radius;
           
            double temp = radius * sin(CandVel[i][1] * time) / d;
            theta = acos(temp);     // acosは-1から1までの値を受け取り0からpiまでの値を返す

            if(CandVel[i][1] < 0) theta -= M_PI; 
            
            PredictTraj_r.back().back().push_back(time);
            PredictTraj_r.back().back().push_back(d);
            PredictTraj_r.back().back().push_back(theta);
    
            // LOG_Traj.push_back(time);
            // LOG_Traj.push_back(d);
            // LOG_Traj.push_back(theta);

            // for (int i = 0; i < LOG_Traj.size(); i++)
            // {
            //     log_traj_file << LOG_Traj[i] << ',';
            // }
            // log_traj_file << std::endl;

            p = np;
            time += dt_traj;
        }

        // #pragma region 
        // std::ofstream d_theta_file("d_theta_file.dat");

        // // 候補軌道の数に対する繰り返し
        // for(int i=0; i<PredictTraj_r.size();i++){
        //     // 軌道内の各時刻に対する繰り返し
        //     for(int j=0;j<PredictTraj_r[i].size();j++)
        //     {
        //         for (int k = 0; k < 3; k++)
        //         {
        //             d_theta_file << PredictTraj_r[i][j][k]<<" ";
        //         }
        //         d_theta_file << std::endl;
        //     }

        //     d_theta_file << std::endl << std::endl;
        // }
        // d_theta_file.close();
        // #pragma endregion

        // log_traj_file << std::endl;
        // LOG_Traj.clear();

        //ROS_INFO("TrajSize%d", PredictTraj.back().size());
    }

    //ROS_INFO("candidate size is %d",CandVel.size());
    //ROS_INFO("finish cal traj\n");

    // joyの軌道の予測
    double time = dt_traj;
    position p = cal_nowp(sensor.odom);
    while (time <= PredictTime)
    {
        // 位置の更新
        position np;
        np = robot_model(p, sensor.joy_cmd_vel[0], sensor.joy_cmd_vel[1], dt_traj);
        //ROS_INFO("push1");
        // timeとpをJoy_PredictTrajに格納する処理
        Joy_PredictTraj.push_back(std::vector<double>());
        Joy_PredictTraj.back().push_back(time);
        Joy_PredictTraj.back().push_back(np.x);
        Joy_PredictTraj.back().push_back(np.y);
        Joy_PredictTraj.back().push_back(np.sin_th);
        Joy_PredictTraj.back().push_back(np.cos_th);
        p = np;
        time += dt_traj;
    }
}

void MyDWA::DWAloop()
{
    ROS_INFO("control loop start.");

    ros::Rate rate(looprate);
    auto start_time = std::chrono::system_clock::now();
    bool plot_flag = false;

    //g.open();

    while (ros::ok())
    {
        ros::spinOnce();
        //g.screen(-120.,-1.,120.,10.);
        
        ROS_INFO("get DWA loop.");

        auto now = std::chrono::system_clock::now();
        auto dur = now - start_time;
        auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
        LOG.push_back((double)msec/1000);

        if (sensor.latest_scan.ranges.size() == 0)
        {
            ROS_INFO("no LRF data.waiting...");
            usleep(1000);
        }
        else
        {

            check_joy();

            LOG.push_back(sensor.odom.twist.twist.linear.x);
            LOG.push_back(sensor.odom.twist.twist.angular.z);
            LOG.push_back(sensor.joy_cmd_vel[0]);
            LOG.push_back(sensor.joy_cmd_vel[1]);

            //　say_time("check joy",now);

            // sensor.detect_line(sensor.latest_scan);
            // for(int i=0; i<sensor.lines.size();i++){
            //     visualization_msgs::MarkerArray linemarkers = sensor.lines[i].make_edge_marker(sensor.center,sensor.latest_scan,sensor.odom.pose);
            //     pub_marker_array(linemarkers);
            // }


            // cal_Dist2ではこれを用いてｄ＿Uを計算するのでコメントしてはならない
            visualization_msgs::MarkerArray obsmarkers = sensor.cal_obs(sensor.latest_scan, 4, 80, sensor.odom.pose);
            // pub_marker_array(obsmarkers);
            
            if (sensor.odom.twist.twist.linear.x >= -0.5)
            {
                cal_DWA();
                LOG.push_back(CandVel.size());
                // say_time("cal DWA",now);

                cal_predict_position();
                //say_time("predict position",now);

                #ifdef PABLODWA
                //cal_Dist();
                cal_Dist2();
                //say_time("cal dist", now);
                

                LOG.push_back(cal_average_d_U(CandVel));

                // double D = cal_vel_sat();

                double D = 1;

                int index = cal_J_sharedDWA(D);
                //say_time("cal J", now);
                
                // 最終的に選択した軌道のマーカ、joyのマーカーと、予測軌道のマーカを作成、表示する
                visualization_msgs::MarkerArray markers = make_traj_marker_array(index);
                pub_marker_array(markers);
                #endif

                #ifdef MYDWA
                search_LRF_Traj(sensor.latest_scan,PredictTraj_r,spec.robot_rad);
                #endif

                #ifdef ISSHARED

                #ifdef PABLODWA
                if (sensor.joy_cmd_vel[0] >= 0)
                {
                    vel.linear.x = CandVel[index][0];
                    vel.angular.z = CandVel[index][1];
                }
                #endif

                #ifdef MYDWA
                // MyDWA
                if (sensor.joy_cmd_vel[0] >= 0)
                {
                    std::cout << "pubvel (" << CandVel[opt_index][0] << ", " << CandVel[opt_index][1] << ")" <<std::endl;
                    vel.linear.x = CandVel[opt_index][0];
                    vel.angular.z = CandVel[opt_index][1];
                }
                #endif
                // MyDWA
                #endif

                // ROS_INFO("pubvel:%f,%f,d_U=%f.", vel.linear.x, vel.angular.z, CandVel[index][2]);


                double distance;
                if (sensor.latest_scan.ranges[sensor.center] == 0)distance=10;
                else distance = sensor.latest_scan.ranges[sensor.center];
                LOG.push_back(distance);

                if (plot_flag)
                {
                    // <matplotlib> //
                    //plot_d_deg();
                    //plot_predict_traj();
                    //g.line(1,1,10,10,"red");
                    //g.line(1,10,20,1,"Yellow");
                    //g.line(100,100,10,10);

                    // <gnuplot> //
                    plot_gnuplot(gp);
                }
                say_time("plot",now);
            }

        }

        pub_cmd.publish(vel);

        // ROS_INFO("pub vel.\n");

        clear_vector();
        //say_time("clear vector", now);

        rate.sleep();
        //say_time("after waiting",now);

        for (int i = 0; i < LOG.size(); i++)
        {
            logfile << LOG[i] << ',';
        }
        logfile << std::endl;
        LOG.clear();
        //g.clear();

        // なにかのキーが押されていることの判定
        if (kbhit())
        {
            int c;
            c = getchar();
            printf("'%c'を押しました。\n ループを抜けるには「e」、プロットを開始するなら「p」、プロットを終えるなら「q」\n", c);
            putchar(c);
            if(c == 'e') break;
            else if(c == 'p') plot_flag = true;
            else if(c == 'q') plot_flag = false;
        }
        ROS_INFO("\n");
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "my_robo_drive");
    //my_robo robot;
    MyDWA robot;

    robot.logfile.open("/home/kitajima/catkin_ws/src/my_robo/my_robo_simulation/log/log.csv");
    //log_traj_file.open("/home/kitajima/catkin_ws/src/my_robo/my_robo_simulation/log/log_traj.csv");

    gp = popen("gnuplot -persist", "w");
    fprintf(gp, "set multiplot\n");
    fprintf(gp, "set xrange [-2:2]\n");
    fprintf(gp, "set yrange [-0.2:2]\n");
    fprintf(gp, "set xlabel \"theta\"\n");
    fprintf(gp, "set ylabel \"distance\"\n");


    std::string logRowName = "timestep,Now vel,now ang,joy vel,joy ang,num cand,ave d_U,pub d_U,velscore,angcore,cost,distance";
    robot.logfile << logRowName << std::endl;

    //logRowName = "time,d,theta";
    //log_traj_file << logRowName;

    // robot.controlloop();
    robot.DWAloop();

    robot.logfile.close();
    //log_traj_file.close();
    //robot.g.close();
    pclose(gp);

    return 0;
}
