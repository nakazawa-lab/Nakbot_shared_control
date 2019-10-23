#include "my_robo_simulation/MyDWA.h"
#include "my_robo_simulation/my_robo_util.h"
#include "my_robo_simulation/kdtree.h"
#include <limits>
#include <chrono>
#include <cassert>

using namespace std;

double MyDWA::cal_Dist(MyPoint query, int idx)
{
    return sqrt((LRFpoints[idx][0] - query[0]) * (LRFpoints[idx][0] - query[0]) + (LRFpoints[idx][1] - query[1]) * (LRFpoints[idx][1] - query[1]));
}


double cal_coll_thres(double r0, double th0, double r, double th){
    return sqrt(r*r + r0*r0 - 2*r*r0*cos(th - th0));
}

double MyDWA::cal_head_cost_pro(int candIdx){
    double cost = abs(CandVel[candIdx][1] - sensor.joy_cmd_vel[1]) / (spec.z_max_ang);

    // cost = sqrt(1 - (1 - cost) * (1 - cost));

    if(isnan(cost)){
        std::cout << "isnan ang h cost" << std::endl;
        cost = 0;
    }
    //cout << "head_cost: " << cost <<endl; 
    return cost;
}

double MyDWA::cal_vel_cost_pro(int candIdx){
    double cost = abs(CandVel[candIdx][0] - sensor.joy_cmd_vel[0]) / spec.x_max_vel; 

    // cost = sqrt(1 - (1 - cost) * (1 - cost));

    if(isnan(cost)) {
        std::cout << "isnan vel h cost" << std::endl;
        cost =0;
    }
    //cout << "vel_h_cost: " << cost <<endl;
    return cost;
}

<<<<<<< HEAD
void MyDWA::cal_dist_sep(){
    double tmp_dist,tmp_ang;
    double dist=10000,ang=10000;
    bool iscol_flag=false;

    // 衝突検知のためのループ
    for (int i = 0; i < CandVel.size(); i++)
    {
        iscol_flag=false;
        dist_lin_ang.push_back(vector<double>());

        for (int j = 0; j < PredictTraj_r[i].size(); j++)
        {
            tmp_dist = cal_lincost_sep_(i, j);

            // if (tmp_dist < dist)
            // {
            //     dist = tmp_dist;
            // }
            if (tmp_dist < spec.robot_rad)
            {
                isCollision.push_back(true);
                //collisionTime.push_back(j * dt_traj);
                dist = 0;
                //ang = cal_angcost_sep_(i, PredictTraj_r[i].size() - 1); // これも0にするべきなのか？
                ang = abs(PredictTraj_r[i][j][2]) / spec.z_max_ang;
                iscol_flag=true;
                break;
            }
        }
        if(!iscol_flag)
        {
            isCollision.push_back(false);
            //collisionTime.push_back(numeric_limits<double>::max());
            ang = cal_angcost_sep_(i, PredictTraj_r[i].size() - 1);
            dist = cal_lincost_sep_(i, PredictTraj_r[i].size() - 1);
        }
        dist_lin_ang[i].push_back(dist / (abs(CandVel[i][0] * thres_vel_time)));
        dist_lin_ang[i].push_back(abs(ang * point_scale_th / (CandVel[i][1] * thres_ang_time)));
    }
}

double MyDWA::cal_lincost_sep_(int i,int j){
    double tmp_dist,dist = 10000;
    int scanId;

    // PredictTraj_r[i][j][2]に最も近いkを求める
    for (int k = 0; k < sensor.latest_scan.ranges.size(); k++)
    {
        // ang方向の最近傍点を求め、そのときのd方向の距離を出す
        tmp_dist = abs(PredictTraj_r[i][j][2] - sensor.index_to_rad(k));

        if(isnan(tmp_dist)){
            tmp_dist=0;
        }
        if(tmp_dist < dist){
            scanId=k;
            dist = tmp_dist;
        }
    }

    // cout << "考えている軌道とその時刻(" << CandVel[i][0] << "," << CandVel[i][1] << ") ," << "t=" << j*dt_traj << endl;
    // cout << "Predicttraj[i][j][2]:" << PredictTraj_r[i][j][2] << ", k=" << scanId << ", rad=" << sensor.index_to_rad(scanId) <<endl;

    dist = abs(PredictTraj_r[i][j][1] - sensor.latest_scan.ranges[scanId]);

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
            cout << "cost: " << cost << endl;
            cout << endl;
        }
        cout << "cand" << i <<endl;
    }
    opt_index = idx_temp;
    cout << "opt idx is" << opt_index << endl << "vel:"  << CandVel[opt_index][0] << " ang: " << CandVel[opt_index][1] <<endl;
}

// cal_dist_sepと似ている
void MyDWA::cal_dist_0925(int candId)
{

}

void MyDWA::cal_opt_0925(){
    double temp_cost, cost = 100000;
=======
void MyDWA::cal_opt_0930(){
    double temp_cost, cost = 10000000;
>>>>>>> 0930
    double head_h_cost_tmp, vel_h_cost_tmp;
    double idx_temp;

    for (int i = 0; i < CandVel.size(); i++)
    {

        // // distを0から1までの間にクリッピングする処理
        // if (dists[i] > 1)
        // {
        //     dists[i] = 1;
        // }

        // distを0から1までの間にクリッピングする処理
        if (dist_lin_ang[i][0] > 1)
        {
            dist_lin_ang[i][0] = 1;
        }
        if (dist_lin_ang[i][1] > 1)
        {
            //cout << "dist_lin_ang[i][1]: " << dist_lin_ang[i][1] << endl;
            dist_lin_ang[i][1] = 1;
        }

        head_h_cost_tmp = cal_head_cost(i);
        //head_h_cost_tmp = cal_head_cost_pro(i);
        vel_h_cost_tmp = cal_vel_cost(i);

<<<<<<< HEAD

        double linsafe = dist_lin_ang[i][0]*dist_lin_ang[i][0];
        double angsafe = dist_lin_ang[i][1]*dist_lin_ang[i][1];
        // double linsafe = dist_lin_ang[i][0];
        // double angsafe = dist_lin_ang[i][1];
=======
        double linsafe = dist_lin_ang[i][0]*dist_lin_ang[i][0]*dist_lin_ang[i][0];
        double angsafe = dist_lin_ang[i][1]*dist_lin_ang[i][1]*dist_lin_ang[i][1];
        //double linsafe = dist_lin_ang[i][0];
        //double angsafe = dist_lin_ang[i][1];
>>>>>>> 0930

        double linadm = 1 - linsafe;
        double angadm = 1 - angsafe;

        // temp_cost = (1 - dist_lin_ang[i][0]) + (1 - dist_lin_ang[i][1]) + (k_velocity * dist_lin_ang[i][0]* vel_h_cost_tmp + k_heading * dist_lin_ang[i][1] * head_h_cost_tmp);
        
        temp_cost = linadm + angadm + (k_velocity * linsafe * vel_h_cost_tmp + k_heading * angsafe * head_h_cost_tmp);

<<<<<<< HEAD
            cout << "CandVel " << i << ":(" << CandVel[i][0] << ", " << CandVel[i][1] << ")" << endl;
            cout << "joy vel: " << sensor.joy_cmd_vel[0] << " " << sensor.joy_cmd_vel[1] << endl;
            // cout << "k_vel * linnormdist * velcost: " << k_velocity * lin_normdists[candIdx] * vel_h_cost  <<endl;
            // cout << "k_head * angnormdist * headcost: " << k_heading * ang_normdists[candIdx] * head_h_cost  <<endl;
            cout << "angsafe: " << angsafe << " head_h_cost: " << head_h_cost_tmp << endl;
            cout << "linsafe: " << linsafe << " vel_h_cost: " << vel_h_cost_tmp << endl;
            cout << "k_vel*angsafe*vel_h_cost: " << k_velocity * angsafe * vel_h_cost_tmp << endl;
            cout << "k_ang*linsafe*ang_h_cost: " << k_heading * linsafe * head_h_cost_tmp<< endl;
            //cout << "linnormDist  + angnormDist: " << dist_lin_ang[i][0] + dist_lin_ang[i][1] << endl;
            cout << "cost: " << temp_cost << endl;
            cout << endl;
=======
            // cout << "CandVel " << i << ":(" << CandVel[i][0] << ", " << CandVel[i][1] << ")" << endl;
            // cout << "joy vel: " << sensor.joy_cmd_vel[0] << " " << sensor.joy_cmd_vel[1] << endl;
            // // cout << "k_vel * linnormdist * velcost: " << k_velocity * lin_normdists[candIdx] * vel_h_cost  <<endl;
            // // cout << "k_head * angnormdist * headcost: " << k_heading * ang_normdists[candIdx] * head_h_cost  <<endl;
            // cout << "angsafe: " << angsafe << " head_h_cost: " << head_h_cost_tmp << endl;
            // cout << "linsafe: " << linsafe << " vel_h_cost: " << vel_h_cost_tmp << endl;
            // cout << "k_vel*angsafe*vel_h_cost: " << k_velocity * angsafe * vel_h_cost_tmp << endl;
            // cout << "k_ang*linsafe*ang_h_cost: " << k_heading * linsafe * head_h_cost_tmp<< endl;
            // //cout << "linnormDist  + angnormDist: " << dist_lin_ang[i][0] + dist_lin_ang[i][1] << endl;
            // cout << "cost: " << temp_cost << endl;
            // cout << endl;
>>>>>>> 0930

        //make_mylog(linadm,linsafe,angadm,angsafe,vel_h_cost_tmp,head_h_cost_tmp,temp_cost,i);

        if (temp_cost < cost)
        {
            cost = temp_cost;
            idx_temp = i;           


            selected.linadm = linadm;
            selected.linsafe = linsafe;
            selected.angadm = angadm;
            selected.angsafe = angsafe;
            selected.vel = CandVel[i][0];
            selected.ang = CandVel[i][1];
            selected.vel_h_cost = vel_h_cost_tmp;
            selected.head_h_cost = head_h_cost_tmp;
            selected.cost = cost;
        }
    }
    //cout <<"finish cal cost" <<endl;

    // mylogfile << endl;
    opt_index = idx_temp;

<<<<<<< HEAD

    // あまりにきけんなときは停止する
    if(cost > 0.99){
        CandVel.push_back(vector<double>());
        CandVel.back().push_back(0.0);
        CandVel.back().push_back(CandVel[opt_index][1]);
        opt_index = CandVel.size()-1;
        cout << "update cand. cand size:" << CandVel.size() <<endl;
    }

    cout << "opt idx is" << opt_index << endl
         << "vel:" << CandVel[opt_index][0] << " ang: " << CandVel[opt_index][1] << endl;
}

void MyDWA::kd_tree_0925(){
    cout << "kdtree" <<endl;
    cout << spec.robot_rad <<endl;
=======
    // あまりにきけんなときは停止する
    if(cost > 0.999){
        CandVel.push_back(vector<double>());
        dist_lin_ang.push_back(vector<double>());
        cout << "update cand. cand size:" << CandVel.size() <<endl;
        CandVel.back().push_back(0.0);
        CandVel.back().push_back(CandVel[opt_index][1]);
        CandVel.back().push_back(numeric_limits<double>::quiet_NaN());
        opt_index = CandVel.size()-1;
 
        dist_lin_ang[opt_index].push_back(numeric_limits<double>::quiet_NaN());
        dist_lin_ang[opt_index].push_back(numeric_limits<double>::quiet_NaN());
        dist_lin_ang[opt_index].push_back(99999999);
        
        selected.vel = CandVel[opt_index][0];
        selected.ang = CandVel[opt_index][1];
    }

    //cout << "dist_lin_ang[opt].size " << dist_lin_ang[opt_index].size() <<endl;
    //cout << "dist_lin_ang[opt][2] " <<dist_lin_ang[opt_index][2] <<endl;   

    //cout << "opt idx is" << opt_index << endl
     //    << "vel:" << CandVel[opt_index][0] << " ang: " << CandVel[opt_index][1] << endl;


    auto now = std::chrono::system_clock::now();
    auto dur = now - start_time;
    auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    mylogfile << (double)(msec/1000.0) << "," << sensor.odom.pose.pose.position.x << "," << sensor.odom.pose.pose.position.y << "," << selected.linadm 
    << "," << selected.linsafe << "," << selected.angadm << "," << selected.angsafe << "," << selected.vel_h_cost << "," 
    << selected.head_h_cost << "," << selected.cost << "," << selected.vel << "," << selected.ang 
    << "," << sensor.joy_cmd_vel[0] << "," << sensor.joy_cmd_vel[1]<< endl;
}

void MyDWA::kd_tree_0930(){
>>>>>>> 0930
    for (int i = 0; i < sensor.point_num; i++)
    {
        LRFpoints.push_back(MyPoint(sensor.index_to_rad(i) * point_scale_th, sensor.latest_scan.ranges[i] * point_scale_d));
    }

    kdt::KDTree<MyPoint> LRFkdtree(LRFpoints);
   // cout << "make kd tree" <<endl;

    MyPoint query;

    double tmp_dist, tmp_ang;
    double dist,lin, ang;
    vector<double> nearest_points;
    int tmp_scan_id,scan_id_nearest;
    int traj_id_nearest;
    double coll_th;

    for (int candId = 0; candId < CandVel.size(); candId++)
    {
        nearest_points.clear();
        dist_lin_ang.push_back(vector<double>());
        dist=numeric_limits<double>::max();
        int traj_size = PredictTraj_r[candId].size();
        //dist_bynorm = sqrt((CandVel[candId][0] * thres_vel_time * point_scale_d) * (CandVel[candId][0] * thres_vel_time* point_scale_d) + (CandVel[candId][1] * thres_ang_time* point_scale_th) * (CandVel[candId][1] * thres_ang_time* point_scale_d)); 

        for (int traj_id = 0; traj_id < traj_size; traj_id++)
        {
<<<<<<< HEAD
            
            tmp_dist = cal_lincost_sep_(candId, traj_id);
            nearest_points.push_back(tmp_dist);
        }

        double nearestDist = *min_element(nearest_points.begin(), nearest_points.end());
        if(isnan(nearestDist)) nearestDist = numeric_limits<double>::infinity();
        int nearestTrajId = (int)distance(nearest_points.begin(),min_element(nearest_points.begin(),nearest_points.end()));

        //cout << "CandVel " << candId << ":(" << CandVel[candId][0] << ", " << CandVel[candId][1] << ")"  << "  nearest Id:" << nearestId << " nearenst:" << nearest <<endl;

        if (nearestDist < spec.robot_rad)
        {
            isCollision.push_back(true);
            collisionTime.push_back(dt_traj * nearestTrajId);

            dist = abs(PredictTraj_r[candId][nearestTrajId][1] / (CandVel[candId][0] * thres_vel_time));
            ang = abs(PredictTraj_r[candId][nearestTrajId][2] / (CandVel[candId][1] * thres_ang_time));
            //ang = abs(PredictTraj_r[candId][nearestTrajId][2]) / (1);
            dist_lin_ang[candId].push_back(dist);
            dist_lin_ang[candId].push_back(abs(ang));
            //dist_lin_ang[candId].push_back(numeric_limits<int>::quiet_NaN());

            // cout << "CandVel " << candId << ":(" << CandVel[candId][0] << ", " << CandVel[candId][1] << ")" << endl;
            // cout << "PredictTraj_r_dist:" << PredictTraj_r[candId][nearestTrajId][1] << " _ang:" << PredictTraj_r[candId][nearestTrajId][2] << endl;
            // cout << "dist:" << dist << " ang:" << ang << endl;
            // cout << endl;
        }
        else
        {
             isCollision.push_back(false);
            // collisionTime.push_back(numeric_limits<double>::max());

            // // PredictTrajをクエリのMyPointに変換
            // query[0] = PredictTraj_r[candId][traj_size - 1][2] * point_scale_th;
            // query[1] = PredictTraj_r[candId][traj_size - 1][1] * point_scale_d;
            // //cout << "query[0]:" << query[0] << " query[1]:" << query[1] << endl;
            // //ROS_INFO("query[0]: %f, query[1]:%f", query[0], query[1]);

            // if (isnan(query[0]))
            //     query[0] = 0;
            // if (isnan(query[1]))
            //     query[1] = 0;

            // int temp_scan_idx = LRFkdtree.nnSearch(query);

            // double lindist = abs(LRFpoints[temp_scan_idx][1] - query[1]);
            // double angdist = abs(LRFpoints[temp_scan_idx][0] - query[0]);

            // // dist_lin_ang[candId].push_back(lindist / abs(spec.x_max_vel * thres_vel_time));
            // // dist_lin_ang[candId].push_back(angdist / abs(spec.z_max_ang * thres_ang_time));
            // dist_lin_ang[candId].push_back(lindist / 1);
            // dist_lin_ang[candId].push_back(angdist / 1);
            // dist_lin_ang[candId].push_back(temp_scan_idx);

            dist_lin_ang[candId].push_back(1);
            dist_lin_ang[candId].push_back(1);
=======
            query[0] = PredictTraj_r[candId][traj_id][2] * point_scale_th;
            query[1] = PredictTraj_r[candId][traj_id][1] * point_scale_d;

            tmp_scan_id = LRFkdtree.nnSearch(query);

            tmp_dist = sqrt((sensor.index_to_rad(tmp_scan_id) * point_scale_th - query[0]) * (sensor.index_to_rad(tmp_scan_id) * point_scale_th - query[0]) + (sensor.latest_scan.ranges[tmp_scan_id]*point_scale_d- query[1]) * (sensor.latest_scan.ranges[tmp_scan_id]*point_scale_d - query[1])  );

            // if (tmp_dist < dist)
            // {
            //     dist = tmp_dist;
            //     traj_id_nearest = traj_id;
            //     scan_id_nearest = tmp_scan_id;
            // }
            // tmp_dist = cal_lincost_sep_(candId, traj_id);
            // nearest_points.push_back(tmp_dist);
            if (cal_coll_thres(sensor.latest_scan.ranges[tmp_scan_id], sensor.index_to_rad(tmp_scan_id), PredictTraj_r[candId][traj_id][1], PredictTraj_r[candId][traj_id][2]) < spec.robot_rad)
            {
                isCollision.push_back(true);
                lin = abs(PredictTraj_r[candId][traj_id][1] / (CandVel[candId][0] * thres_vel_time));
                ang = abs(PredictTraj_r[candId][traj_id][2] / (CandVel[candId][1] * thres_ang_time));
                cout << "CandVel " << candId << ":(" << CandVel[candId][0] << ", " << CandVel[candId][1] << ")" << endl;
                cout << "PredictTraj_r: " << PredictTraj_r[candId][traj_id][1] << " " << PredictTraj_r[candId][traj_id][2] <<endl;
                cout << "sensor: " << sensor.latest_scan.ranges[tmp_scan_id] << endl;
                cout << "lin:" <<lin << " ang:" <<ang <<endl;
                cout << "traj_id:" <<traj_id << " scan_id:" <<tmp_scan_id <<endl;
                cout <<endl;
                dist_lin_ang[candId].push_back(lin);
                dist_lin_ang[candId].push_back(ang);
                dist_lin_ang[candId].push_back(tmp_scan_id);
                break;
            }
            else if (traj_id == traj_size - 1)
            {
                isCollision.push_back(false);
                dist_lin_ang[candId].push_back(1);
                dist_lin_ang[candId].push_back(1);
                dist_lin_ang[candId].push_back(tmp_scan_id);
            }
>>>>>>> 0930
        }
    }
    assert(isCollision.size() == CandVel.size());
    assert(dist_lin_ang.size() == CandVel.size());
    //cout << "finish cal dist" <<endl;
}

void MyDWA::proposed_0930(){
    kd_tree_0930();
    say_time("after kd_tree",loop_start_time);
    cal_opt_0930();
}

void MyDWA::Proposed(){
    // cal_dist_sep();
    // cal_cost_sep();
    // search_LRF_Traj();
    proposed_0930();
}

<<<<<<< HEAD
visualization_msgs::Marker MyDWA::make_nearest_LRF_marker(int optId){
=======

visualization_msgs::Marker MyDWA::make_nearest_LRF_marker(int optId){
    cout << "make nearest LRF marker optId "  << optId  <<endl;
>>>>>>> 0930
    position p = sensor.index_to_pos(optId);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = (ros::Duration)0.5;

<<<<<<< HEAD
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
=======
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
>>>>>>> 0930
    marker.pose.position.x=p.x;
    marker.pose.position.y=p.y;
    marker.pose.position.z=0;
    marker.pose.orientation.x=0;
    marker.pose.orientation.y=0;
    marker.pose.orientation.z=0;
    marker.pose.orientation.w=1;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;
    // pub_mark.publish(marker);
    return marker;
}


void MyDWA::clear_vector()
{
    std::vector<double> tmp_double;
    std::vector<bool> tmp_bool;

    int i=0;
    std::vector<std::vector<double>>().swap(CandVel);

    std::vector<std::vector<std::vector<double>>>().swap(PredictTraj);

    vector<bool>().swap(isCollision);

    std::vector<std::vector<double>>().swap(Joy_PredictTraj);

    vector<line>().swap(sensor.lines);

    std::vector<std::vector<std::vector<double>>>().swap(PredictTraj_r);

    vector<double>().swap(lin_normdists);

    vector<double>().swap(ang_normdists);

    vector<vector<double>>().swap(dist_lin_ang);

    LRFkdtree.clear();

    vector<MyPoint>().swap(LRFpoints);

    vector<double>().swap(LOG);
    // myDWA.clear();
    // ROS_INFO("candsize:%d",CandVel.size());
    // ROS_INFO("predict:%d",PredictTraj.size());
    // ROS_INFO("isCollisiton:%d",isCollision.size());
}

void MyDWA::record_param(){
    std::string property = "dt,dt_traj,PredictTime,looprate,k_head,k_vel,thres_vel_time,thres_ang_time,d_scale,th_scale";
    logfile << property << std::endl;

    logfile << dt << "," << dt_traj << "," << PredictTime << "," << looprate << "," << k_heading << "," << k_velocity 
            << "," << thres_vel_time << "," << thres_ang_time << "," << point_scale_d << "," << point_scale_th << endl<<endl;

    // std::string logRowName = "timestep,Now vel,now ang,joy vel,joy ang,num cand,ave d_U,pub d_U,velscore,angcore,cost,distance";
    // logfile << logRowName << std::endl;

    std::string logRowName = "timestep,pos.x,pos.y,adm,safe,vel_h_cost,ang_h_cost,cost,cal_vel.v,cal_val.w,joy_v,joy_w";
    logfile << logRowName << std::endl;


    mylogfile << property << std::endl;
    mylogfile << dt << "," << dt_traj << "," << PredictTime << "," << looprate << "," << k_heading << "," << k_velocity 
            << "," << thres_vel_time << "," << thres_ang_time << "," << point_scale_d << "," << point_scale_th << endl<<endl;

    // std::string mylogRowName = "joyvel,joyang,CandVel,CandAng,linadm,linsafe,angadm,angsafe,vel_h_cost,ang_h_cost,cost";
    // mylogfile << mylogRowName << std::endl;

    std::string mylogRowName = "timestep,pos.x,pos.y,linadm,linsafe,angadm,angsafe,vel_h_cost,ang_h_cost,cost,cal_vel.v,cal_val.w,joy_v,joy_w";
    mylogfile << mylogRowName << std::endl;
}

void MyDWA::make_mylog(double linadm, double linsafe, double angadm, double angsafe, double vel_h_cost_tmp, double head_h_cost_tmp, double temp_cost, int i)
{
    mylogfile << sensor.joy_cmd_vel[0] << "," << sensor.joy_cmd_vel[1] << "," << CandVel[i][0] << "," << CandVel[i][1] << "," << linadm << "," << linsafe
              << "," << angadm << "," << angsafe << "," << vel_h_cost_tmp << "," << head_h_cost_tmp << "," << temp_cost << endl;
}

void MyDWA::make_mylog_perloop(double time)
{
    cout << selected.angadm << " " << selected.head_h_cost << endl;
    mylogfile << time << "," << sensor.odom.pose.pose.position.x << "," << sensor.odom.pose.pose.position.y << "," << selected.linadm << "," << selected.linsafe
              << "," << selected.angadm << "," << selected.angsafe << "," << selected.vel_h_cost << "," << selected.head_h_cost << "," << selected.cost << ","
               << selected.vel << "," << selected.ang << "," << sensor.joy_cmd_vel[0] << "," << sensor.joy_cmd_vel[1]<< endl;
}