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

double cal_coll_thres(double r0, double th0, double r, double th)
{
    return sqrt(r * r + r0 * r0 - 2 * r * r0 * cos(th - th0));
}

double MyDWA::cal_head_cost_pro(int candIdx)
{
    double cost = fabs(CandVel[candIdx][1] - sensor.joy_cmd_vel[1]) / (spec.z_max_ang);

    // cost = sqrt(1 - (1 - cost) * (1 - cost));

    if (isnan(cost))
    {
        std::cout << "isnan ang h cost" << std::endl;
        cost = 0;
    }
    //cout << "head_cost: " << cost <<endl;
    return cost;
}

double MyDWA::cal_vel_cost_pro(int candIdx)
{
    double cost = fabs(CandVel[candIdx][0] - sensor.joy_cmd_vel[0]) / spec.x_max_vel;

    // cost = sqrt(1 - (1 - cost) * (1 - cost));

    if (isnan(cost))
    {
        std::cout << "isnan vel h cost" << std::endl;
        cost = 0;
    }
    //cout << "vel_h_cost: " << cost <<endl;
    return cost;
}

void MyDWA::cal_opt()
{
    double temp_cost, cost = 10000000;
    double head_h_cost_tmp, vel_h_cost_tmp;
    double idx_temp;
    double arctan2_h;

    if (sensor.joy_cmd_vel[1] == 0 && sensor.joy_cmd_vel[0] == 0)
    {
        arctan2_h = 0;
    }
    else
    {
        arctan2_h = atan2(sensor.joy_cmd_vel[0], sensor.joy_cmd_vel[1]);
        //std::cout << "human arctan2 " << arctan2_h << std::endl;
    }

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

        head_h_cost_tmp = cal_head_cost(i, arctan2_h);
        //head_h_cost_tmp = cal_head_cost_pro(i);
        vel_h_cost_tmp = cal_vel_cost(i);

        double linsafe = pow(dist_lin_ang[i][0], LINSAFE_MULTIPLIER);
        double angsafe = pow(dist_lin_ang[i][1], ANGSAFE_MULTIPLIER);
        double linadm = 1 - linsafe;
        double angadm = 1 - angsafe;

        // temp_cost = (1 - dist_lin_ang[i][0]) + (1 - dist_lin_ang[i][1]) + (k_velocity * dist_lin_ang[i][0]* vel_h_cost_tmp + k_heading * dist_lin_ang[i][1] * head_h_cost_tmp);

        temp_cost = linadm + angadm + (k_velocity * linsafe * vel_h_cost_tmp + k_heading * angsafe * head_h_cost_tmp);

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

            selected.lindist = dist_lin_ang[i][0];
            selected.angdist = dist_lin_ang[i][1];
        }
    }
    //cout <<"finish cal cost" <<endl;

    // mylogfile << endl;
    opt_index = idx_temp;

    // あまりにきけんなときは停止する
    if (cost > 0.999)
    {
        CandVel.push_back(vector<double>());
        dist_lin_ang.push_back(vector<double>());
        cout << "danger. new cand size:" << CandVel.size() << endl;
        CandVel.back().push_back(0.0);
        CandVel.back().push_back(CandVel[opt_index][1]);
        CandVel.back().push_back(numeric_limits<double>::quiet_NaN());
        opt_index = CandVel.size() - 1;

        dist_lin_ang[opt_index].push_back(numeric_limits<double>::quiet_NaN());
        dist_lin_ang[opt_index].push_back(numeric_limits<double>::quiet_NaN());
        dist_lin_ang[opt_index].push_back(99999999);

        selected.vel = CandVel[opt_index][0];
        selected.ang = CandVel[opt_index][1];
    }

    // cout << "dist_lin_ang[opt].size " << dist_lin_ang[opt_index].size() <<endl;
    // cout << "dist_lin_ang[opt][2] " <<dist_lin_ang[opt_index][2] <<endl;

    // cout << "opt idx is" << opt_index << endl
    //     << "vel:" << CandVel[opt_index][0] << " ang: " << CandVel[opt_index][1] << endl;

    auto now = std::chrono::system_clock::now();
    auto dur = now - start_time;
    auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    mylogfile << (double)(msec / 1000.0) << "," << sensor.odom.pose.pose.position.x << "," << sensor.odom.pose.pose.position.y << "," << selected.linadm
              << "," << selected.linsafe << "," << selected.angadm << "," << selected.angsafe << "," << selected.vel_h_cost << ","
              << selected.head_h_cost << "," << selected.cost << "," << selected.vel << "," << selected.ang
              << "," << sensor.joy_cmd_vel[0] << "," << sensor.joy_cmd_vel[1] << "," << selected.lindist << "," << selected.angdist
              << "," << sensor.odom.twist.twist.linear.x << "," << sensor.odom.twist.twist.angular.z << endl;
}

void MyDWA::kd_tree()
{
    bool IsNoObs = true;
    double tmp_dist, tmp_ang;
    double dist, lin, ang;
    int tmp_scan_id, scan_id_nearest;
    int traj_id_nearest;

    for (int i = 0; i < sensor.point_num; i++)
    {
        if (!isinf(sensor.latest_scan.ranges[i]))
        {
            LRFpoints.push_back(MyPoint(sensor.index_to_rad(i), sensor.latest_scan.ranges[i]));
            thinout_scan_range.push_back(sensor.latest_scan.ranges[i]);
            thinout_scan_ang.push_back(sensor.index_to_rad(i));
            IsNoObs = false;
        }
    }

    if (!IsNoObs)
    {
        kdt::KDTree<MyPoint> LRFkdtree(LRFpoints);
        MyPoint query;
        //cout << "make kd tree" << endl;

        for (int candId = 0; candId < CandVel.size(); candId++)
        {
            dist_lin_ang.push_back(vector<double>());
            int traj_size = PredictTraj_r[candId].size();
           

            for (int traj_id = 0; traj_id < traj_size; traj_id++)
            {
                query[0] = PredictTraj_r[candId][traj_id][2];
                query[1] = PredictTraj_r[candId][traj_id][1];
                
                tmp_scan_id = LRFkdtree.nnSearch(query);
                // cout << "query:(" << query[0]  << ", " << query[1] << ")" <<endl;
                // cout << "scan id:" << tmp_scan_id << endl;
                // cout << "thinout_range:" << thinout_scan_range[tmp_scan_id] << " ang:" << thinout_scan_ang[tmp_scan_id] << endl;
                tmp_dist = cal_coll_thres(thinout_scan_range[tmp_scan_id], thinout_scan_ang[tmp_scan_id], PredictTraj_r[candId][traj_id][1], PredictTraj_r[candId][traj_id][2]);
                if (tmp_dist < spec.ROBOT_RAD)
                {
                    isCollision.push_back(true);
                    lin = fabs(PredictTraj_r[candId][traj_id][1] / (CandVel[candId][0] * thres_vel_time));
                    ang = fabs(PredictTraj_r[candId][traj_id][2] / (CandVel[candId][1] * thres_ang_time));
                    cout << "CandVel " << candId << ":(" << CandVel[candId][0] << ", " << CandVel[candId][1] << ")" << endl;
                    cout << "PredictTraj_r: " << PredictTraj_r[candId][traj_id][1] << " " << PredictTraj_r[candId][traj_id][2] <<endl;
                    //cout << "sensor: " << thinout_scan_range[tmp_scan_id] << endl;
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
                    //cout << "4" <<endl;
                    isCollision.push_back(false);
                    dist_lin_ang[candId].push_back(1);
                    dist_lin_ang[candId].push_back(1);
                    dist_lin_ang[candId].push_back(tmp_scan_id);
                }
            }
        }
    }
    else
    {
        for (int candId = 0; candId < CandVel.size(); candId++)
        {
            dist_lin_ang.push_back(vector<double>());
            dist_lin_ang[candId].push_back(1);
            dist_lin_ang[candId].push_back(1);
            dist_lin_ang[candId].push_back(99999999);

            isCollision.push_back(false);
        }
    }
    assert(isCollision.size() == CandVel.size());
    assert(dist_lin_ang.size() == CandVel.size());
    cout << "finish cal dist" <<endl;
}

void MyDWA::Proposed()
{
    kd_tree();
    say_time("kd_tree", loop_start_time);
    cal_opt();
}

visualization_msgs::Marker MyDWA::make_nearest_LRF_marker(int optId)
{
    cout << "make nearest LRF marker optId " << optId << endl;
    position p = sensor.index_to_pos(optId);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = (ros::Duration)0.5;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.pose.position.x = p.x;
    marker.pose.position.y = p.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;
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

    vector<float>().swap(thinout_scan_range);

    vector<float>().swap(thinout_scan_ang);
}

void MyDWA::record_param()
{
    std::string property = "dt,dt_traj,PredictTime,looprate,k_head,k_vel,thres_vel_time,thres_ang_time";
    logfile << property << std::endl;

    logfile << dt << "," << dt_traj << "," << PredictTime << "," << looprate << "," << k_heading << "," << k_velocity
            << "," << thres_vel_time << "," << thres_ang_time << endl
            << endl;

    // std::string logRowName = "timestep,Now vel,now ang,joy vel,joy ang,num cand,ave d_U,pub d_U,velscore,angcore,cost,distance";
    // logfile << logRowName << std::endl;

    std::string logRowName = "timestep,pos.x,pos.y,adm,safe,vel_h_cost,ang_h_cost,cost,cal_vel.v,cal_val.w,joy_v,joy_w,now_v,now_w";
    logfile << logRowName << std::endl;

    mylogfile << property << std::endl;
    mylogfile << dt << "," << dt_traj << "," << PredictTime << "," << looprate << "," << k_heading << "," << k_velocity
              << "," << thres_vel_time << "," << thres_ang_time << endl
              << endl;

    // std::string mylogRowName = "joyvel,joyang,CandVel,CandAng,linadm,linsafe,angadm,angsafe,vel_h_cost,ang_h_cost,cost";
    // mylogfile << mylogRowName << std::endl;

    std::string mylogRowName = "timestep,pos.x,pos.y,linadm,linsafe,angadm,angsafe,vel_h_cost,ang_h_cost,cost,cal_vel.v,cal_val.w,joy_v,joy_w,lindist,angdist,now_v,now_w";
    mylogfile << mylogRowName << std::endl;
}

// void MyDWA::make_mylog(double linadm, double linsafe, double angadm, double angsafe, double vel_h_cost_tmp, double head_h_cost_tmp, double temp_cost, int i)
// {
//     mylogfile << sensor.joy_cmd_vel[0] << "," << sensor.joy_cmd_vel[1] << "," << CandVel[i][0] << "," << CandVel[i][1] << "," << linadm << "," << linsafe
//               << "," << angadm << "," << angsafe << "," << vel_h_cost_tmp << "," << head_h_cost_tmp << "," << temp_cost << endl;
// }

// void MyDWA::make_mylog_perloop(double time)
// {
//     cout << selected.angadm << " " << selected.head_h_cost << endl;
//     mylogfile << time << "," << sensor.odom.pose.pose.position.x << "," << sensor.odom.pose.pose.position.y << "," << selected.linadm << "," << selected.linsafe
//               << "," << selected.angadm << "," << selected.angsafe << "," << selected.vel_h_cost << "," << selected.head_h_cost << "," << selected.cost << ","
//                << selected.vel << "," << selected.ang << "," << sensor.joy_cmd_vel[0] << "," << sensor.joy_cmd_vel[1]<< endl;
// }
