#include "my_robo_simulation/MyDWA.h"
#include "my_robo_simulation/my_robo_util.h"
#include "my_robo_simulation/kdtree.h"
#include <limits>
#include <chrono>
#include <cassert>

using namespace std;
extern double cal_euclid(double x0, double y0, double x1, double y1);
extern FILE *gp;

double cal_coll_thres(double r0, double th0, double r, double th)
{
    return sqrt(r * r + r0 * r0 - 2 * r * r0 * cos(th - th0));
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

    for (int i = 0; i < CandVel_v.size(); i++)
    {
        // distを0から1までの間にクリッピングする処理
        if (dist_lin_ang[i][0] > 1)
        {
            dist_lin_ang[i][0] = 1;
        }
        if (dist_lin_ang[i][1] > 1)
        {
            dist_lin_ang[i][1] = 1;
        }

        head_h_cost_tmp = cal_head_cost(i, arctan2_h);
        vel_h_cost_tmp = cal_vel_cost(i);

        double linsafe = pow(dist_lin_ang[i][0], LINSAFE_MULTIPLIER);
        double angsafe = pow(dist_lin_ang[i][1], ANGSAFE_MULTIPLIER);
        double linadm = 1 - linsafe;
        double angadm = 1 - angsafe;

        // temp_cost = (1 - dist_lin_ang[i][0]) + (1 - dist_lin_ang[i][1]) + (k_velocity * dist_lin_ang[i][0]* vel_h_cost_tmp + k_heading * dist_lin_ang[i][1] * head_h_cost_tmp);

        temp_cost = linadm + angadm + (k_velocity * linsafe * vel_h_cost_tmp + k_heading * angsafe * head_h_cost_tmp);

        // cout << "CandVel " << i << ":(" << CandVel_v[i] << ", " << CandVel_w[i] << ")" << endl;
        // cout << "joy vel: " << sensor.joy_cmd_vel[0] << " " << sensor.joy_cmd_vel[1] << endl;
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
            selected.vel = CandVel_v[i];
            selected.ang = CandVel_w[i];
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
    if (cost > 1)
    {
        double min_vel = *std::min_element(CandVel_v.begin(), CandVel_v.end());
        std::cout << min_vel << std::endl;
        if(min_vel != 0) std::cout <<"min_vel != 0" <<std::endl <<std::endl;;


        dist_lin_ang.push_back(vector<double>());
        cout << "danger. new cand size:" << CandVel_v.size() << " cost:" << cost << endl;
        CandVel_v.push_back(min_vel);
        CandVel_w.push_back(CandVel_w[opt_index]);
        opt_index = CandVel_v.size() - 1;

        double time = 0.0;
        while (time < PredictTime)
        {
            PredictTraj.push_back(std::vector<std::vector<double>>());
            // timeとpをPredictTrajに格納する処理
            PredictTraj.back().push_back(std::vector<double>());
            PredictTraj.back().back().push_back(time);
            PredictTraj.back().back().push_back(0);
            PredictTraj.back().back().push_back(0);
            // sin とcosは使わないので入れない
            time += dt_traj;
        }

        dist_lin_ang[opt_index].push_back(dist_lin_ang[opt_index - 1][0]);
        dist_lin_ang[opt_index].push_back(dist_lin_ang[opt_index - 1][1]);
        dist_lin_ang[opt_index].push_back(dist_lin_ang[opt_index - 1][2]);

        selected.vel = CandVel_v[opt_index];
        selected.ang = CandVel_w[opt_index];
    }

    // cout << "dist_lin_ang[opt].size " << dist_lin_ang[opt_index].size() <<endl;
    // cout << "dist_lin_ang[opt][2] " <<dist_lin_ang[opt_index][2] <<endl;

    // cout << "opt idx is" << opt_index << endl
    //      << "vel:" << CandVel_v[opt_index] << " ang: " << CandVel_w[opt_index] << endl;

    // auto now = std::chrono::system_clock::now();
    // auto dur = now - start_time;
    // auto msec = std::chrono::duration_cast<std::chrono::milliseconds>(dur).count();
    // mylogfile << (double)(msec / 1000.0) << "," << sensor.odom.pose.pose.position.x << "," << sensor.odom.pose.pose.position.y << "," << selected.linadm
    //           << "," << selected.linsafe << "," << selected.angadm << "," << selected.angsafe << "," << selected.vel_h_cost << ","
    //           << selected.head_h_cost << "," << selected.cost << "," << selected.vel << "," << selected.ang
    //           << "," << sensor.joy_cmd_vel[0] << "," << sensor.joy_cmd_vel[1] << "," << selected.lindist << "," << selected.angdist
    //           << "," << sensor.odom.twist.twist.linear.x << "," << sensor.odom.twist.twist.angular.z << endl;
}

void MyDWA::kd_tree()
{
    bool IsNoObs = true;
    double tmp_dist, tmp_ang;
    double dist, lin, ang;
    int tmp_scan_id, scan_id_nearest;
    int traj_id_nearest;
    double x, y;

    //visualization_msgs::MarkerArray marker_array;
    //marker_array.markers.resize(684);
    int k=0;

    for (int i = 0; i < sensor.point_num; i++)
    {
        if (!isinf(sensor.latest_scan.ranges[i]))
        {
            if(IsREAL){
                if(sensor.latest_scan.ranges[i]>0.06){
                    position p = sensor.index_to_pos(i);
                    LRFpoints.push_back(MyPoint(p.x, p.y));
                    thinout_scan_x.push_back(p.x);
                    thinout_scan_y.push_back(p.y);
                    IsNoObs = false;

                    //cout << "in kdtree " << i << " " << p.x << " " << p.y << " "<<sensor.latest_scan.ranges[i] << " " << sensor.index_to_rad(i)*RAD2DEG<<endl;
                }
                else{
                    //cout << "is 0. index:" << i <<endl;
                }
            }
            else{
                position p = sensor.index_to_pos(i);
                LRFpoints.push_back(MyPoint(p.x, p.y));
                thinout_scan_x.push_back(p.x);
                thinout_scan_y.push_back(p.y);
                IsNoObs = false;
            }
            // marker_array.markers[k].header.frame_id = "/odom";
            // marker_array.markers[k].header.stamp = ros::Time::now();
            // marker_array.markers[k].ns = "LRF";
            // marker_array.markers[k].id = k;
            // marker_array.markers[k].lifetime = (ros::Duration)(PUB_TRAJ_MARKER_PER_LOOP * dt);

            // // marker_array.markers[j].type = visualization_msgs::Marker::CUBE;
            // marker_array.markers[k].type = visualization_msgs::Marker::SPHERE;
            // marker_array.markers[k].action = visualization_msgs::Marker::ADD;
            // marker_array.markers[k].scale.x = 0.1;
            // marker_array.markers[k].scale.y = 0.1;
            // marker_array.markers[k].scale.z = 0.1;
            // marker_array.markers[k].pose.position.x = p.x;
            // marker_array.markers[k].pose.position.y = p.y;
            // marker_array.markers[k].pose.position.z = 0;
            // marker_array.markers[k].pose.orientation.x = 0;
            // marker_array.markers[k].pose.orientation.y = 0;
            // marker_array.markers[k].pose.orientation.z = 0;
            // marker_array.markers[k].pose.orientation.w = 1;

            // marker_array.markers[k].color.r = 0.0f;
            // marker_array.markers[k].color.g = 1.0f;
            // marker_array.markers[k].color.b = 0.0f;
            // marker_array.markers[k].color.a = 1.0f;
            // k++;
        } 
    }
    //pub_marker_array(marker_array);
    //plot_scan_gnuplot(gp,thinout_scan_x,thinout_scan_y);

    if (!IsNoObs)
    {
        kdt::KDTree<MyPoint> LRFkdtree(LRFpoints);
        MyPoint query;
        //cout << "make kd tree" << endl;

        for (int candId = 0; candId < CandVel_v.size(); candId++)
        {
            dist_lin_ang.push_back(vector<double>());
            int traj_size = PredictTraj[candId].size();

            for (int traj_id = 0; traj_id < traj_size; traj_id++)
            {
                //query[0] = PredictTraj_r[candId][traj_id][2];
                //query[1] = PredictTraj_r[candId][traj_id][1];
                query[0] = PredictTraj[candId][traj_id][1];
                query[1] = PredictTraj[candId][traj_id][2];

                tmp_scan_id = LRFkdtree.nnSearch(query);
                // cout << "query:(" << query[0]  << ", " << query[1] << ")" <<endl;
                //cout << "scan id:" << tmp_scan_id << endl;
                //cout << "nearest x:" << thinout_scan_x[tmp_scan_id] << " y:" << thinout_scan_y[tmp_scan_id] << endl;
                //tmp_dist = cal_coll_thres(thinout_scan_range[tmp_scan_id], thinout_scan_ang[tmp_scan_id], PredictTraj_r[candId][traj_id][1], PredictTraj_r[candId][traj_id][2]);
                tmp_dist = cal_euclid(thinout_scan_x[tmp_scan_id], thinout_scan_y[tmp_scan_id], PredictTraj[candId][traj_id][1], PredictTraj[candId][traj_id][2]);
                if (tmp_dist < spec.ROBOT_RAD)
                {
                    isCollision.push_back(true);
                    lin = fabs(CandVel_v[candId] * PredictTraj[candId][traj_id][0] / (CandVel_v[candId] * PredictTime));
                    ang = fabs(CandVel_w[candId] * PredictTraj[candId][traj_id][0] / (CandVel_w[candId] * PredictTime));
                    // cout << "CandVel " << candId << ":(" << CandVel_v[candId] << ", " << CandVel_w[candId] << ")" << endl;
                    // cout << "PredictTraj: " << PredictTraj[candId][traj_id][1] << " " << PredictTraj[candId][traj_id][2] <<endl;
                    // //cout << "sensor: " << thinout_scan_range[tmp_scan_id] << endl;
                    // cout << "lin:" <<lin << " ang:" <<ang <<endl;
                    // cout << "traj_id:" <<traj_id << " scan_id:" <<tmp_scan_id <<endl;
                    // cout << endl;
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
            }
        }
    }
    else
    {
        for (int candId = 0; candId < CandVel_v.size(); candId++)
        {
            dist_lin_ang.push_back(vector<double>());
            dist_lin_ang[candId].push_back(1);
            dist_lin_ang[candId].push_back(1);
            dist_lin_ang[candId].push_back(99999999);

            isCollision.push_back(false);
        }
    }
    assert(isCollision.size() == CandVel_v.size());
    assert(dist_lin_ang.size() == CandVel_v.size());
}

void MyDWA::Proposed()
{
    kd_tree();
    //say_time("kd_tree", loop_start_time);
    cal_opt();
}

visualization_msgs::Marker MyDWA::make_nearest_LRF_marker(float x, float y)
{
    visualization_msgs::Marker marker;
    std::cout << "start make marker" << std::endl;
    marker.header.frame_id = "/odom";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = (ros::Duration)0.5;

    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
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
    std::cout << "fin make marker" << std::endl;
    return marker;
}

void MyDWA::clear_vector()
{
    using namespace std;
    vector<double> tmp_double;
    vector<bool> tmp_bool;

    vector<bool>().swap(isCollision);
    vector<MyPoint>().swap(LRFpoints);
    vector<double>().swap(CandVel_v);
    vector<double>().swap(CandVel_w);
    vector<double>().swap(d_U);
    vector<float>().swap(thinout_scan_x);
    vector<float>().swap(thinout_scan_y);
    vector<vector<double>>().swap(dist_lin_ang);
    vector<std::vector<double>>().swap(Joy_PredictTraj);
    vector<std::vector<std::vector<double>>>().swap(PredictTraj);
    LRFkdtree.clear();
}

void MyDWA::record_param()
{
    std::string property = "dt,dt_traj,PredictTime,looprate,k_head,k_vel";
    logfile << property << std::endl;

    logfile << dt << "," << dt_traj << "," << PredictTime << "," << looprate << "," << k_heading << "," << k_velocity
            << endl
            << endl;

    // std::string logRowName = "timestep,Now vel,now ang,joy vel,joy ang,num cand,ave d_U,pub d_U,velscore,angcore,cost,distance";
    // logfile << logRowName << std::endl;

    std::string logRowName = "timestep[s],pos.x,pos.y,adm,safe,vel_h_cost,ang_h_cost,cost,cal_vel.v,cal_val.w,joy_v,joy_w,now_v,now_w,cal_time[ms],nearest[m],direction[rad]bb";
    logfile << logRowName << std::endl;

    mylogfile << property << std::endl;
    mylogfile << dt << "," << dt_traj << "," << PredictTime << "," << looprate << "," << k_heading << "," << k_velocity
              << endl
              << endl;

    // std::string mylogRowName = "joyvel,joyang,CandVel,CandAng,linadm,linsafe,angadm,angsafe,vel_h_cost,ang_h_cost,cost";
    // mylogfile << mylogRowName << std::endl;

    std::string mylogRowName = "timestep[s],pos.x,pos.y,linadm,linsafe,angadm,angsafe,vel_h_cost,ang_h_cost,cost,cal_vel.v,cal_val.w,joy_v,joy_w,lindist,angdist,now_v,now_w,cal_time[ms],nearest[m],direction[rad]aa";
    mylogfile << mylogRowName << std::endl;
}
