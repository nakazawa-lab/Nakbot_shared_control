#include "my_robo_simulation/my_robo_drive.h"
#include "my_robo_simulation/my_robo_util.h"
#include"my_robo_simulation/kdtree.h"
#include <limits>

void MyDWA::cal_Dist(){

}

double MyDWA::cal_Dist(MyPoint query, int idx){
    return sqrt((LRFpoints[idx][0] - query[0]) * (LRFpoints[idx][0] - query[0]) + (LRFpoints[idx][1] - query[1]) * (LRFpoints[idx][1] - query[1])); 
}

// kd木によって、障害物を示す点群の中から、任意の点(d,theta)に対して最短距離となる点のインデックスを求める
int MyDWA::kd_tree_nnSearch(const MyPoint query){
    ROS_INFO("kd tree nn search");
    // usage; nearest neibor search
    // const MyPoint query(xx, yy);

    // kdtreeを構成した直後では実行できるのになぜここでエラーが生じる?
    int idx = LRFkdtree.nnSearch(query);

    ROS_INFO("fin nnSerach in func");
    return idx;
}

void MyDWA::make_kd_tree(sensor_msgs::LaserScan& scan){
    ROS_INFO("make kd tree");
    int point_num = scan.ranges.size();
    // LRFpoints.resize(point_num);
    std::cout<<"point_num:"<<point_num<<std::endl;

    for(int i=0; i< point_num; i++){
        LRFpoints.push_back(MyPoint(scan.angle_increment * (i - scan.ranges.size()/2) * RAD2DEG, scan.ranges[i]));
    }

    std::cout<<"LRFPoints size" << LRFpoints.size() <<std::endl;

    kdt::KDTree<MyPoint> kdtree(LRFpoints);
    ROS_INFO("test test");
    MyPoint query(1,1);
    LRFkdtree = kdtree;
    int idx = LRFkdtree.nnSearch(query);
    ROS_INFO("test idx: %d",idx);
    ROS_INFO("test fin");
}

void MyDWA::search_LRF_Traj(sensor_msgs::LaserScan& latest_scan, std::vector<std::vector<std::vector<double>>>& PredictTrajs){
    ROS_INFO("serch_LRF_trajs");
    int traj_num = PredictTrajs.size();
    ROS_INFO("pretraj_r size:%d",traj_num);

    // LRFについてのkd木を作る
    make_kd_tree(latest_scan);

    // 各軌道に対してLRFと軌道の再接近点点を求める。
    for(int i=0; i< traj_num; i++){
        search_LRF_Traj(latest_scan, PredictTrajs[i]);
    }
}

// ここでいうpredictTrajはベクトルの次元が１つ小さくなっており単一軌道を表すことに注意する
void MyDWA::search_LRF_Traj(sensor_msgs::LaserScan& latest_scan, std::vector<std::vector<double>>& PredictTraj){
    ROS_INFO("serch_LRF_traj");
    // trajの各点(各時刻)に対してLRFのkdtreeからNNサーチを行い、最小となる距離とそのインデックスを保持しておく
    double dist = 10000000;
    int scan_idx,traj_idx;
    for(int i=0; i<PredictTraj.size(); i++){
        // PredictTrajをクエリのMyPointに変換
        ROS_INFO("test 1 ");
        MyPoint query(PredictTraj[i][2]* RAD2DEG,PredictTraj[i][1]);
        ROS_INFO("query[0]: %f, query[1]:%f",query[0],query[1]);


/// test code: NANの判定 #include<limits>　が必要
        if (isnan(query[0]))
        {
            std::cout << "is NAN." << std::endl;
            query[0] = 0.1;
        }
        else
        {
            std::cout << "is not NAN." << std::endl;
        }
/// test code 

        int temp_scan_idx = kd_tree_nnSearch(query);
        double temp_dist = cal_Dist(query,scan_idx);
        ROS_INFO("test 2 ");

        if(temp_dist < dist){
            dist = temp_dist;
            scan_idx = temp_scan_idx;
            traj_idx = i;
        }
    }

    scan_indices.push_back(scan_idx);
    traj_indices.push_back(traj_idx);
    cal_lin_ang_Dist(scan_idx,traj_idx,PredictTraj);
}

void MyDWA::cal_lin_ang_Dist(int scan_idx,int traj_idx,std::vector<std::vector<double>>& PredictTraj){
    ROS_INFO("cal_lin_ang_dist");
    lin_dists.push_back(abs(LRFpoints[scan_idx][0]-PredictTraj[traj_idx][2]));
    ang_dists.push_back(abs(LRFpoints[scan_idx][1]-PredictTraj[traj_idx][1]));
}