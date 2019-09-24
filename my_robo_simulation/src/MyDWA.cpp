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

    // 保存していたcostベクトルの中から、最小値とそのインデックスを取得
    // 参考コード
    //std::vector<int>::iterator iter = std::max_element(x.begin(), x.end());
    //size_t index = std::distance(x.begin(), iter);

    // 最小コストを満たすインデックスの保持 opt_indexに保持する
    vector<double>::iterator iter = min_element(costs.begin(),costs.end());
    size_t index = distance(costs.begin(),iter);
    opt_index = (int)index;

    cout << "finish search " << endl;
    cout << "opt idx: " << opt_index <<endl;
}

// kdtreeを作り、探索も同時に行う
void MyDWA::kd_tree(sensor_msgs::LaserScan &scan, std::vector<std::vector<std::vector<double>>> &PredictTrajs, double &robot_rad)
{
    ROS_INFO("make kd tree");
    int traj_num = PredictTrajs.size();
    int point_num = scan.ranges.size();

    for (int i = 0; i < point_num; i++)
    {
        LRFpoints.push_back(MyPoint(scan.angle_increment * (i - scan.ranges.size() / 2) * point_scale_th, scan.ranges[i] * point_scale_d * point_scale_d));
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
            //     ROS_INFO("update dist! j = %d", j);
            //     ROS_INFO("temp dist:%f", temp_dist);
            //     ROS_INFO("dist:%f", dist);
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

        //cal_lin_ang_Dist(scan_idx, traj_idx, PredictTrajs[i], robot_rad, i);

        //cal_costs(i);
        cal_costs_0924(i,dist);

        // for (int i = 0; i < LOG_MYDWA.size(); i++)
        // {
        //     mylogfile << LOG_MYDWA[i] << ',';
        // }
        // mylogfile << std::endl;
        // LOG_MYDWA.clear();
    }
}

void MyDWA::cal_lin_ang_Dist(int scan_idx, int traj_idx, std::vector<std::vector<double>> &PredictTraj, double &robot_rad, int candIdx)
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
        
        lin_normDist = lin_dist / (CandVel[candIdx][0] * thres_vel_time);
        ang_normDist = ang_dist / (abs(CandVel[candIdx][1]) * thres_ang_time);
        if (lin_normDist > 1)
            lin_normDist = 1;
        if (ang_normDist > 1)
            ang_normDist = 1;
        if (lin_normDist < 0)
            lin_normDist = 0;
        if (ang_normDist < 0)
            ang_normDist = 0;
    }
    else
    {
        isCollision.push_back(false);
        if (CandVel[candIdx][0] <= 0)
            lin_normDist = 1;
        if (CandVel[candIdx][1] == 0)
            ang_normDist = 1;
        lin_normDist = lin_dist / (CandVel[candIdx][0] * thres_vel_time);
        ang_normDist = ang_dist / (abs(CandVel[candIdx][1]) * thres_ang_time);

        if (isnan(lin_normDist) || CandVel[candIdx][0] < 0)
            lin_normDist = 1;
        if (isnan(ang_normDist))
            ang_normDist = 1;

        if (lin_normDist > 1)
            lin_normDist = 1;
        if (ang_normDist > 1)
            ang_normDist = 1;
        if (lin_normDist < 0)
            lin_normDist = 0;
        if (ang_normDist < 0)
            ang_normDist = 0;
    }
    cout << "CandVel " << candIdx << ":(" << CandVel[candIdx][0] << ", " << CandVel[candIdx][1] << ")" << endl;
    cout << "lin_dist : " << lin_dist << " ang_dist: " << ang_dist << endl;
    cout << "lin_normDist : " << lin_normDist << " ang_normDist: " << ang_normDist << endl;
    //cout << endl;
    lin_normdists.push_back(lin_normDist);
    ang_normdists.push_back(ang_normDist);

    LOG_MYDWA.push_back(lin_normDist);
    LOG_MYDWA.push_back(ang_normDist);
}

// 各軌道に対して呼び出される 0920夜 trajIdxによってコストを決めなくてはならない！繰り返し不要
void MyDWA::cal_costs(int candIdx)
{
    double temp_cost, cost = 10000;
    double head_h_cost,vel_h_cost;
    double head_h_cost_tmp,vel_h_cost_tmp;
    int temp_idx;
    //for(int i=0; i < PredictTraj_r.size();i++){    
        //if(!isCollision[i]){
            head_h_cost_tmp = cal_head_cost(candIdx); 
            //head_h_cost_tmp = cal_head_cost_pro(candIdx);
            vel_h_cost_tmp = cal_vel_cost(candIdx);
            // コストの計算
            temp_cost =  ((1-lin_normdists[candIdx])  + (1 - ang_normdists[candIdx])) + ( k_velocity * lin_normdists[candIdx] * vel_h_cost_tmp + k_heading * ang_normdists[candIdx] * head_h_cost_tmp) ;
            
            cout << "joy vel: " << sensor.joy_cmd_vel[0] << " " << sensor.joy_cmd_vel[1] << endl;
            cout << "1 - linnormdist: " << 1 - lin_normdists[candIdx] <<endl;
            cout << "1 - angnormdist: " << 1 - ang_normdists[candIdx] <<endl;
            // cout << "k_vel * linnormdist * velcost: " << k_velocity * lin_normdists[candIdx] * vel_h_cost  <<endl;
            // cout << "k_head * angnormdist * headcost: " << k_heading * ang_normdists[candIdx] * head_h_cost  <<endl;
            cout << "k_head: " << k_heading << " angnormDist: " <<ang_normdists[candIdx] << " head_h_cost: " << head_h_cost_tmp <<endl;
            cout << "k_vel: " << k_velocity << " linnormDist: " <<lin_normdists[candIdx] << " vel_h_cost: " << vel_h_cost_tmp <<endl;
            cout << "cost: " <<temp_cost <<endl;
            cout << endl;


            // // 最小コストの判定、保持
            // if(temp_cost < cost){
            //     vel_h_cost = vel_h_cost_tmp;
            //     head_h_cost = head_h_cost_tmp;
            //     cout << "update cost traj " << i << ": temp cost=" << temp_cost << " cost=" << cost << endl;
            //     cout << "Candvel: " << CandVel[i][0] << " CandAng:" << CandVel[i][1] << endl; 
            //     cout << "vel_h_cost:" << vel_h_cost << endl;
            //     cout << "head_h_cost:" << head_h_cost << endl;
            //     cout << "lin_normdist:" << lin_normdists[i] << endl;
            //     cout << "ang_normdist:" << ang_normdists[i] << endl;
            //     cout <<endl;
            //     cost = temp_cost;
            //     temp_idx = i;
            // }

        //}
    //}       

    LOG_MYDWA.push_back(vel_h_cost_tmp);
    LOG_MYDWA.push_back(head_h_cost_tmp);
    LOG_MYDWA.push_back(temp_cost);

    costs.push_back(temp_cost);
}

void MyDWA::cal_costs_0924(int candIdx, double dist){
    double temp_cost;
    double head_h_cost, vel_h_cost;
    double head_h_cost_tmp, vel_h_cost_tmp;
    int temp_idx;

    double basedist = (CandVel[candIdx][0]*thres_vel_time) * (CandVel[candIdx][0]*thres_vel_time)  + (CandVel[candIdx][1] * thres_ang_time) * (CandVel[candIdx][1] * thres_ang_time);
    dist /= basedist;
    if(dist > 1){
        dist =1;
    }


    //for(int i=0; i < PredictTraj_r.size();i++){
    //if(!isCollision[i]){
    head_h_cost_tmp = cal_head_cost(candIdx);
    //head_h_cost_tmp = cal_head_cost_pro(candIdx);
    vel_h_cost_tmp = cal_vel_cost(candIdx);
    // コストの計算
    temp_cost = (1 - dist) + dist * (k_velocity * vel_h_cost_tmp + k_heading * head_h_cost_tmp);

    cout << "CandVel " << candIdx << ":(" << CandVel[candIdx][0] << ", " << CandVel[candIdx][1] << ")" << endl;

    cout << "joy vel: " << sensor.joy_cmd_vel[0] << " " << sensor.joy_cmd_vel[1] << endl;
    cout << "1 - ldist: " << 1 - dist << endl;
    // cout << "k_vel * linnormdist * velcost: " << k_velocity * lin_normdists[candIdx] * vel_h_cost  <<endl;
    // cout << "k_head * angnormdist * headcost: " << k_heading * ang_normdists[candIdx] * head_h_cost  <<endl;
    cout << "k_head: " << k_heading << " head_h_cost: " << head_h_cost_tmp << endl;
    cout << "k_vel: " << k_velocity << " vel_h_cost: " << vel_h_cost_tmp << endl;
    cout << "cost: " << temp_cost << endl;
    cout << endl;

    costs.push_back(temp_cost);
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

double MyDWA::cal_head_cost_pro(int candIdx){
    double cost = abs(CandVel[candIdx][1] - sensor.joy_cmd_vel[1]) / spec.z_max_ang;
    if(isnan(cost)){
        std::cout << "isnan ang h cost" << std::endl;
        cost = 0;
    }
    //cout << "head_cost: " << cost <<endl; 
    return cost;
}

void MyDWA::cal_dist_sep(){
    double tmp_dist,tmp_ang;
    double dist=10000,ang=10000;
    bool flag=false;

    // 衝突検知のためのループ
    for (int i = 0; i < CandVel.size(); i++)
    {
        flag=false;
        dist_lin_ang.push_back(vector<double>());

        for (int j = 0; j < PredictTraj_r[i].size(); j++)
        {
            tmp_dist = cal_lincost_sep_(i, j);

            if (tmp_dist < dist)
            {
                dist = tmp_dist;
            }
            if (tmp_dist < spec.robot_rad)
            {
                isCollision.push_back(true);
                collisionTime.push_back(j * dt_traj);
                dist = 0.1;
                //ang = cal_angcost_sep_(i, PredictTraj_r[i].size() - 1); // これも0にするべきなのか？
                ang = 0;
                flag=true;
                break;
        }
            }
        if(!flag)
        {
            isCollision.push_back(false);
            collisionTime.push_back(numeric_limits<double>::max());
            ang = cal_angcost_sep_(i, PredictTraj_r[i].size() - 1);
            dist = cal_lincost_sep_(i, PredictTraj_r[i].size() - 1);
        }
        dist_lin_ang[i].push_back(dist / (abs(CandVel[i][0] * thres_vel_time)));
        dist_lin_ang[i].push_back(abs(ang * point_scale_th / (CandVel[i][1] * thres_ang_time)));
    }
}

double MyDWA::cal_lincost_sep_(int i,int j){
    double tmp_dist,dist = 10000;
    for (int k = 0; k < sensor.latest_scan.ranges.size(); k++)
    {
        // dist についての最近傍点を求め、コストに変換する
        tmp_dist = abs(PredictTraj_r[i][j][1] - sensor.latest_scan.ranges[k]);

        if(tmp_dist < dist){
            dist = tmp_dist;
        }
    }

    return dist;
}

double MyDWA::cal_angcost_sep_(int i, int j){
    double tmp_ang,ang = 10000;
    for (int k = 0; k < sensor.latest_scan.ranges.size(); k++)
    {
        // ang についての最近傍点を求め、コストに変換する
        tmp_ang = abs(PredictTraj_r[i][j][2] - sensor.index_to_rad(k));

        if(tmp_ang < ang){
            ang = tmp_ang;
            //cout << "update ang. ang=" << ang <<endl;
        }
    }

    return ang;
}

void MyDWA::cal_cost_sep(){
    double temp_cost,cost=100000;
    double head_h_cost_tmp,vel_h_cost_tmp;
    double idx_temp;

    for(int i=0;i<CandVel.size();i++){

        // distを0から1までの間にクリッピングする処理
        if(dist_lin_ang[i][0] > 1){
            dist_lin_ang[i][0] = 1;
        }
        if(dist_lin_ang[i][1] > 1){
            cout << "dist_lin_ang[i][1]: " << dist_lin_ang[i][1] <<endl;
            dist_lin_ang[i][1] = 1;
        }

        //head_h_cost_tmp = cal_head_cost(i);
        head_h_cost_tmp = cal_head_cost_pro(i);
        vel_h_cost_tmp = cal_vel_cost(i);

        // temp_cost = (1 - dist_lin_ang[i][0]) + (1 - dist_lin_ang[i][1]) + (k_velocity * dist_lin_ang[i][0]* vel_h_cost_tmp + k_heading * dist_lin_ang[i][1] * head_h_cost_tmp);
            temp_cost = (1 - dist_lin_ang[i][0]) +  (1 - dist_lin_ang[i][1]) + (dist_lin_ang[i][0] +  dist_lin_ang[i][1]) * (k_velocity * vel_h_cost_tmp + k_heading * head_h_cost_tmp);

        if(temp_cost < cost){
            cost = temp_cost;
            idx_temp = i;

            cout << "CandVel " << i << ":(" << CandVel[i][0] << ", " << CandVel[i][1] << ")" << endl;
            cout << "joy vel: " << sensor.joy_cmd_vel[0] << " " << sensor.joy_cmd_vel[1] << endl;
            // cout << "k_vel * linnormdist * velcost: " << k_velocity * lin_normdists[candIdx] * vel_h_cost  <<endl;
            // cout << "k_head * angnormdist * headcost: " << k_heading * ang_normdists[candIdx] * head_h_cost  <<endl;
            cout << "angnormDist: " << dist_lin_ang[i][1] << " head_h_cost: " << head_h_cost_tmp << endl;
            cout << "linnormDist: " << dist_lin_ang[i][0] << " vel_h_cost: " << vel_h_cost_tmp << endl;
            cout << "linnormDist  + angnormDist: " << dist_lin_ang[i][0] + dist_lin_ang[i][1] << endl;
            cout << "exp(- collisionTime^2: " << exp(-1 * collisionTime[i] * collisionTime[i]) << endl;
            cout << "cost: " << temp_cost << endl;
            cout << endl;
        }
    }
    opt_index = idx_temp;
    cout << "opt idx is" << opt_index << endl << "vel:"  << CandVel[opt_index][0] << " ang: " << CandVel[opt_index][1] <<endl;
}

void MyDWA::Proposed(){
    // cal_dist_sep();
    // cal_cost_sep();
    search_LRF_Traj(sensor.latest_scan,PredictTraj_r,spec.robot_rad);
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
    costs.clear();
    dist_lin_ang.clear();
    collisionTime.clear();
    LRFkdtree.clear();
    LRFpoints.clear();
    // myDWA.clear();
    // ROS_INFO("candsize:%d",CandVel.size());
    // ROS_INFO("predict:%d",PredictTraj.size());
    // ROS_INFO("isCollisiton:%d",isCollision.size());
}