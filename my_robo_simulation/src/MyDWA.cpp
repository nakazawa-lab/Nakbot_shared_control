#include "my_robo_simulation/MyDWA.h"
#include "my_robo_simulation/my_robo_util.h"
#include "my_robo_simulation/kdtree.h"
#include <limits>

using namespace std;

// これだけ外から呼び出せば中の関数を使ってkd木の探索ができる
void MyDWA::search_LRF_Traj(sensor_msgs::LaserScan &latest_scan, std::vector<std::vector<std::vector<double>>> &PredictTrajs, double robot_rad)
{
    // LRFについてのkd木を作り、探索を行う
    kd_tree(latest_scan, PredictTrajs, robot_rad);
}

// kdtreeを作り、探索も同時に行う
void MyDWA::kd_tree(sensor_msgs::LaserScan &scan, std::vector<std::vector<std::vector<double>>> &PredictTrajs, double &robot_rad)
{
    ROS_INFO("make kd tree");
    int traj_num = PredictTrajs.size();
    int point_num = scan.ranges.size();

    for (int i = 0; i < point_num; i++)
    {
        LRFpoints.push_back(MyPoint(scan.angle_increment * (i - scan.ranges.size() / 2) * point_scale_th, scan.ranges[i] * point_scale_d));
    }

    kdt::KDTree<MyPoint> LRFkdtree(LRFpoints);

    //cout << "軌道のサイズ" << traj_num << endl;

    // 各軌道に対してLRFと軌道の再接近点点を求める。
    for (int i = 0; i < traj_num; i++)
    {
        // search_LRF_Traj(scan, PredictTrajs[i]);
        MyPoint query;

        // ROS_INFO("serch_LRF_traj");
        // trajの各点(各時刻)に対してLRFのkdtreeからNNサーチを行い、最小となる距離とそのインデックスを保持しておく
        double dist = 10000000;
        int scan_idx, traj_idx;
        // std::cout << "軌道の番号:" << i << std::endl;
        // cout<< "候補速度:(" << CandVel[i][0] << ", " << CandVel[i][1] << ")" << endl;
        //cout << "時刻のサイズ:" << PredictTrajs[i].size() << endl;
        for (int j = 0; j < PredictTrajs[i].size(); j++)
        {
            //cout << "時刻の番号:" << j << " 時刻:" << j * dt_traj <<endl;
            // PredictTrajをクエリのMyPointに変換
            query[0] = PredictTrajs[i][j][2] * point_scale_th;
            query[1] = PredictTrajs[i][j][1] * point_scale_d;
            // cout << "query[0]:" << query[0] << " query[1]:" << query[1] << endl;
            // cout << endl;
            //ROS_INFO("query[0]: %f, query[1]:%f", query[0], query[1]);
            /// test code: NANの判定 #include<limits>　が必要
            if (isnan(query[0]))
            {
                // std::cout << "is NAN." << std::endl;
                // cout << "query[0]:" << query[0] << " query[1]:" << query[1] << endl;
                query[0] = 0;
            }
            if (isnan(query[1]))
            {
                // std::cout << "is NAN." << std::endl;
                // cout << "query[0]:" << query[0] << " query[1]:" << query[1] << endl;
                query[1] = 0;
            }
            /// test code

            int temp_scan_idx = LRFkdtree.nnSearch(query);
            // std::cout << "temp_scan_idx:" << temp_scan_idx << std::endl;
            double temp_dist = sqrt((LRFpoints[temp_scan_idx][0] - query[0]) * (LRFpoints[temp_scan_idx][0] - query[0]) + (LRFpoints[temp_scan_idx][1] - query[1]) * (LRFpoints[temp_scan_idx][1] - query[1]));
            // std::cout << "temp dist :"<< temp_dist << std::endl;

            if (temp_dist < dist)
            {
                // ROS_INFO("update dist! j = %d", j);
                // ROS_INFO("temp dist:%f", temp_dist);
                // ROS_INFO("dist:%f", dist);
                dist = temp_dist;

                // ROS_INFO("scan_idx:%d", scan_idx);
                // ROS_INFO("temp_scan_idx:%d", temp_scan_idx);
                scan_idx = temp_scan_idx;
                traj_idx = j;
            }
        }
        scan_indices.push_back(scan_idx);
        traj_indices.push_back(traj_idx);

        LOG_MYDWA.push_back(sensor.joy_cmd_vel[0]);
        LOG_MYDWA.push_back(sensor.joy_cmd_vel[1]);
        LOG_MYDWA.push_back(CandVel[i][0]);
        LOG_MYDWA.push_back(CandVel[i][1]);
        LOG_MYDWA.push_back(dist);

        cal_lin_ang_Dist(scan_idx, traj_idx, PredictTrajs[i], robot_rad, i);

        // コストの計算を行い、候補軌道の中から最適なもののインデックスを計算する
        cal_costs();

        for (int i = 0; i < LOG_MYDWA.size(); i++)
        {
            mylogfile << LOG_MYDWA[i] << ',';
        }
        mylogfile << std::endl;
        LOG_MYDWA.clear();
    }
}

void MyDWA::cal_lin_ang_Dist(int scan_idx, int traj_idx, std::vector<std::vector<double>> &PredictTraj, double &robot_rad, int i)
{
    double lin_normDist, ang_normDist;
    // ROS_INFO("cal_lin_ang_dist");
    // ROS_INFO("scan_idx:%d", scan_idx);
    // ROS_INFO("traj_idx:%d", traj_idx);
    // ROS_INFO("LRFpoints[scan_idx][0]:%f", LRFpoints[scan_idx][0]);
    // ROS_INFO("LRFpoints[traj_idx][0]:%f", PredictTraj[traj_idx][0]);
    double lin_dist = abs(LRFpoints[scan_idx][1] - PredictTraj[traj_idx][1]) * point_scale_d;
    double ang_dist = abs(LRFpoints[scan_idx][0] - PredictTraj[traj_idx][2]) * point_scale_th;
    if (lin_dist < robot_rad)
    {
        // cout << endl;
        // cout << "collision for CandVel " << i << ":(" << CandVel[i][0] << ", " << CandVel[i][1] << ")"
        //      << " traj_idx:" << traj_idx << endl;
        // cout << "lin dist" << lin_dist << endl;
        // cout << "robot_rad" << robot_rad << endl;
        // cout << "LRFpoints[scan_idx][0] trans_rad: " << LRFpoints[scan_idx][0] << " PredictTraj[traj_idx][2]: relative rad:" << PredictTraj[traj_idx][2] << endl;
        // cout << "LRFpoints[scan_idx][1] trand_dist: " << LRFpoints[scan_idx][1] << " PredictTraj[traj_idx][1]: relative dist:" << PredictTraj[traj_idx][1] << endl;
        isCollision.push_back(true);
        
        lin_normDist = lin_dist / (CandVel[i][0] * thres_vel_time);
        ang_normDist = ang_dist / (abs(CandVel[i][1]) * thres_ang_time);
        if(lin_normDist > 1) lin_normDist =1;
        if(ang_normDist >1) ang_normDist =1;
    }
    else
    {
        isCollision.push_back(false);
        if(CandVel[i][0] <= 0) lin_normDist = 1;
        if (CandVel[i][1] == 0) ang_normDist =1;
        lin_normDist = lin_dist / (CandVel[i][0] * thres_vel_time);
        ang_normDist = ang_dist / (abs(CandVel[i][1])* thres_ang_time);

        if (isnan(lin_normDist) || CandVel[i][0] < 0)
            lin_normDist = 1;
        if (isnan(ang_normDist))
            ang_normDist = 1;

        if(lin_normDist > 1) lin_normDist =1;
        if(ang_normDist >1) ang_normDist =1;
    }
    cout << "CandVel " << i << ":(" << CandVel[i][0] << ", " << CandVel[i][1] << ")" << endl;
    cout << "lin_dist : " << lin_dist << " ang_dist: " << ang_dist << endl;
    cout << "lin_normDist : " << lin_normDist << " ang_normDist: " << ang_normDist << endl;
    cout <<endl;
    lin_normdists.push_back(lin_normDist);
    ang_normdists.push_back(ang_normDist);

    LOG_MYDWA.push_back(lin_normDist);
    LOG_MYDWA.push_back(ang_normDist);
}

void MyDWA::cal_costs()
{
    double temp_cost, cost = 10000;
    double head_h_cost,vel_h_cost;
    double head_h_cost_tmp,vel_h_cost_tmp;
    int temp_idx;
    for(int i=0; i < PredictTraj_r.size();i++){
        //if(!isCollision[i]){
            head_h_cost_tmp = cal_head_cost(i); 
            vel_h_cost_tmp = cal_vel_cost(i);
            // コストの計算
            temp_cost = 0.5 * ((1-lin_normdists[i])  + (1 - ang_normdists[i])) + 0.5 * ( k_velocity * lin_normdists[i] * vel_h_cost + k_heading * ang_normdists[i] * head_h_cost);

            // 最小コストの判定、保持
            if(temp_cost < cost){
                vel_h_cost = vel_h_cost_tmp;
                head_h_cost = head_h_cost_tmp;
                cout << "update cost traj " << i << ": temp cost=" << temp_cost << " cost=" << cost << endl;
                cout << "Candvel: " << CandVel[i][0] << " CandAng:" << CandVel[i][1] << endl; 
                cout << "vel_h_cost:" << vel_h_cost << endl;
                cout << "head_h_cost:" << head_h_cost << endl;
                cout << "lin_normdist:" << lin_normdists[i] << endl;
                cout << "ang_normdist:" << ang_normdists[i] << endl;
                cout <<endl;
                cost = temp_cost;
                temp_idx = i;
            }

        //}
    }
    // 最小コストを満たすインデックスの保持 opt_indexに保持する
    opt_index = temp_idx;
    LOG_MYDWA.push_back(vel_h_cost);
    LOG_MYDWA.push_back(head_h_cost);
    LOG_MYDWA.push_back(cost);
}

void MyDWA::cal_Dist()
{
}

double MyDWA::cal_Dist(MyPoint query, int idx)
{
    return sqrt((LRFpoints[idx][0] - query[0]) * (LRFpoints[idx][0] - query[0]) + (LRFpoints[idx][1] - query[1]) * (LRFpoints[idx][1] - query[1]));
}

// kd木によって、障害物を示す点群の中から、任意の点(d,theta)に対して最短距離となる点のインデックスを求める
int MyDWA::kd_tree_nnSearch(const MyPoint query)
{
}

// ここでいうpredictTrajはベクトルの次元が１つ小さくなっており単一軌道を表すことに注意する
void MyDWA::search_LRF_Traj(sensor_msgs::LaserScan &latest_scan, std::vector<std::vector<double>> &PredictTraj)
{
}

void MyDWA::clear_vector()
{
    // ループの最後にはpredicttrajectoryやcmdcandidateなどを消去する
    CandVel.clear();
    PredictTraj.clear();
    isCollision.clear();
    Joy_PredictTraj.clear();
    sensor.lines.clear();
    PredictTraj_r.clear();
    lin_normdists.clear();
    ang_normdists.clear();
    // myDWA.clear();
    // ROS_INFO("candsize:%d",CandVel.size());
    // ROS_INFO("predict:%d",PredictTraj.size());
    // ROS_INFO("isCollisiton:%d",isCollision.size());
}