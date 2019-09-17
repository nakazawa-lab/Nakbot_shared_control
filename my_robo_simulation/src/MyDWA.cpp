#include "my_robo_simulation/my_robo_drive.h"
#include "my_robo_simulation/my_robo_util.h"
#include "my_robo_simulation/kdtree.h"
#include <limits>

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

// kdtreeを作り、探索も同時に行う
void MyDWA::kd_tree(sensor_msgs::LaserScan &scan, std::vector<std::vector<std::vector<double>>> &PredictTrajs)
{
    ROS_INFO("make kd tree");
    int traj_num = PredictTrajs.size();
    int point_num = scan.ranges.size();

    for (int i = 0; i < point_num; i++)
    {
        LRFpoints.push_back(MyPoint(scan.angle_increment * (i - scan.ranges.size() / 2) * RAD2DEG, scan.ranges[i]));
    }

    kdt::KDTree<MyPoint> LRFkdtree(LRFpoints);
    // ROS_INFO("test test");
    // MyPoint query(1, 1);
    // // LRFkdtree = kdtree;
    // int idx = LRFkdtree.nnSearch(query);
    // ROS_INFO("test idx: %d", idx);
    // ROS_INFO("test fin");

    // 各軌道に対してLRFと軌道の再接近点点を求める。
    for (int i = 0; i < traj_num; i++)
    {
        // search_LRF_Traj(scan, PredictTrajs[i]);
        MyPoint query;

        // ROS_INFO("serch_LRF_traj");
        // trajの各点(各時刻)に対してLRFのkdtreeからNNサーチを行い、最小となる距離とそのインデックスを保持しておく
        double dist = 10000000;
        int scan_idx, traj_idx;
        for (int j = 0; j < PredictTrajs[i].size(); j++)
        {
            // PredictTrajをクエリのMyPointに変換
            query[0] = PredictTrajs[i][j][2] * RAD2DEG;
            query[1] = PredictTrajs[i][j][1];
            //ROS_INFO("query[0]: %f, query[1]:%f", query[0], query[1]);

            /// test code: NANの判定 #include<limits>　が必要
            if (isnan(query[0]))
            {
                //std::cout << "is NAN." << std::endl;
                query[0] = 0.1;
            }
            else
            {
                //std::cout << "is not NAN." << std::endl;
            }
            if (isnan(query[1]))
            {
                query[1] = 0.1;
            }
            /// test code

            int temp_scan_idx = LRFkdtree.nnSearch(query);
            // std::cout << "temp_scan_idx:" << temp_scan_idx << std::endl;
            double temp_dist = sqrt((LRFpoints[temp_scan_idx][0] - query[0]) * (LRFpoints[temp_scan_idx][0] - query[0]) + (LRFpoints[temp_scan_idx][1] - query[1]) * (LRFpoints[temp_scan_idx][1] - query[1]));
            // std::cout << "temp dist :"<< temp_dist << std::endl;

            if (temp_dist < dist)
            {
                ROS_INFO("update dist! j = %d", j);
                ROS_INFO("temp dist:%f", temp_dist);
                ROS_INFO("dist:%f", dist);
                dist = temp_dist;

                ROS_INFO("scan_idx:%d", scan_idx);
                ROS_INFO("temp_scan_idx:%d", temp_scan_idx);
                scan_idx = temp_scan_idx;
                traj_idx = j;
            }
        }

        scan_indices.push_back(scan_idx);
        traj_indices.push_back(traj_idx);
        cal_lin_ang_Dist(scan_idx, traj_idx, PredictTrajs[i]);
    }
}

// これだけ外から呼び出せば中の関数を使ってkd木の探索ができる
void MyDWA::search_LRF_Traj(sensor_msgs::LaserScan &latest_scan, std::vector<std::vector<std::vector<double>>> &PredictTrajs)
{
    ROS_INFO("serch_LRF_trajs");

    // LRFについてのkd木を作り、探索を1つずつ行う
    kd_tree(latest_scan, PredictTrajs);
}

// ここでいうpredictTrajはベクトルの次元が１つ小さくなっており単一軌道を表すことに注意する
void MyDWA::search_LRF_Traj(sensor_msgs::LaserScan &latest_scan, std::vector<std::vector<double>> &PredictTraj)
{
}

void MyDWA::cal_lin_ang_Dist(int scan_idx, int traj_idx, std::vector<std::vector<double>> &PredictTraj)
{
    // ROS_INFO("cal_lin_ang_dist");
    // ROS_INFO("scan_idx:%d", scan_idx);
    // ROS_INFO("traj_idx:%d", traj_idx);
    // ROS_INFO("LRFpoints[scan_idx][0]:%f", LRFpoints[scan_idx][0]);
    // ROS_INFO("LRFpoints[traj_idx][0]:%f", PredictTraj[traj_idx][0]);
    lin_dists.push_back(abs(LRFpoints[scan_idx][0] - PredictTraj[traj_idx][2]));
    ang_dists.push_back(abs(LRFpoints[scan_idx][1] - PredictTraj[traj_idx][1]));
}