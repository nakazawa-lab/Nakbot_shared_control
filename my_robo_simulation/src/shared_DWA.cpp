#include "my_robo_simulation/my_robo_drive.h"
#include "my_robo_simulation/my_robo_util.h"

using namespace std;

// 速度コストのsaturation係数を求める
double my_robo::cal_vel_sat()
{
    int num = CandVel_v.size();
    // ROS_INFO("start cal sat.");
    // 予測時刻あとに衝突しないような軌道を探す
    // obsからクリアランスを求める
    double clearance;
    // 予測自国内に衝突しない候補の数を求める
    int count = 0;

    //ROS_INFO("num:%d",num);
    if (!isCollision.empty())
    {

        // 候補速度に対して
        for (int i = 0; i < num; i++)
        {
            //候補速度の中で、衝突しない速度を選ぶ

            if (!isCollision[i])
            {
                count++;
                double dist = 10000000000;
                //ROS_INFO("i=%d",i);

                // Dの計算
                // predictTreajの一番最後と、obsの距離を比べる
                // predictTraj[j-1]とセンサ障  FILE *gp;害物の最大の距離を求める
                //ROS_INFO("sensor_size:%d",sensor.obs.size());
                if (sensor.obs.size() == 0)
                {
                    ROS_INFO("no obs");
                    dist = 8;
                }
                else
                {
                    for (int k = 0; k < sensor.obs.size(); k++)
                    {
                        int a = PredictTraj[i][0].size();

                        // ROS_INFO("trajSize:%d",a);
                        // ROS_INFO("euclid");
                        // ROS_INFO("dist:%f",PredictTraj[i][a-1][0]);
                        // ROS_INFO("dist:%f",PredictTraj[i][a-1][1]);
                        // ROS_INFO("dist:%f",sensor.obs[k][0]);
                        // ROS_INFO("dist:%f",sensor.obs[k][1]);
                        double temp = cal_euclid(PredictTraj[i][a - 1][0], sensor.obs[k][0], PredictTraj[i][a - 1][1], sensor.obs[k][1]);
                        //ROS_INFO("finish euclid. dist:%f",temp);
                        if (temp < dist)
                        {
                            dist = temp;
                        }
                    }
                }
                //ROS_INFO("final dist:%f",dist);
                clearance += dist;
            }
            // }
        }
    }
    //ROS_INFO("number not collision:%d",count);

    // 予測時刻あとの座標はPredictTraj[j-1][x][y][th]
    // クリアランス計算
    // クリアランスをどうやって求めるか？
    double D = clearance / count;
    //ROS_INFO("D:%lf\n",D);
    return D;
}

// 予め計算しておいた何度かずつのSCANの位置に従って距離を計算する
void my_robo::cal_Dist2()
{
    double adm;
    double t = 0.05, time;
    double v, w, r;      // rは曲率半径
    double d_lin, d_ang; // sharedDWAの論文における定義
    double d, th;        // レーザセンサに対応した距離と角度
    double d_U;
    bool IsNoObs = true;
    int tmp_scan_id, scan_id_nearest;
    double tmp_dist;

    for (int i = 0; i < sensor.point_num; i++)
    {
        if (!isinf(sensor.latest_scan.ranges[i]))
        {
            position p = sensor.index_to_pos(i);
            LRFpoints.push_back(MyPoint(p.x,p.y));
            thinout_scan_x.push_back(p.x);
            thinout_scan_y.push_back(p.y);
            IsNoObs = false;
        }
    }

    if (!IsNoObs)
    {
        kdt::KDTree<MyPoint> LRFkdtree(LRFpoints);
        MyPoint query;

        for (int candId = 0; candId < CandVel_v.size(); candId++)
        {
            int traj_size = PredictTraj[candId].size();
            for (int traj_id = 0; traj_id < traj_size; traj_id++)
            {
                query[0]=PredictTraj[candId][traj_id][1];
                query[1]=PredictTraj[candId][traj_id][2];
                
                tmp_scan_id = LRFkdtree.nnSearch(query);
                // cout << "query:(" << query[0]  << ", " << query[1] << ")" <<endl;
                // cout << "scan id:" << tmp_scan_id << endl;
                // cout << "thinout_range:" << thinout_scan_range[tmp_scan_id] << " ang:" << thinout_scan_ang[tmp_scan_id] << endl;
                //tmp_dist = cal_coll_thres(thinout_scan_range[tmp_scan_id], thinout_scan_ang[tmp_scan_id], PredictTraj_r[candId][traj_id][1], PredictTraj_r[candId][traj_id][2]);
                tmp_dist = cal_euclid(thinout_scan_x[tmp_scan_id],thinout_scan_y[tmp_scan_id],PredictTraj[candId][traj_id][1],PredictTraj[candId][traj_id][2]);
                if (tmp_dist < spec.ROBOT_RAD)
                {
                    d_lin=CandVel_v[candId]*PredictTraj[candId][traj_id][0];
                    d_ang=CandVel_w[candId]*PredictTraj[candId][traj_id][1];
                    double v_inev = sqrt(fabs(2 * spec.x_min_acc * d_lin));
                    double w_inev = sqrt(fabs(2 * spec.z_min_acc * d_ang));

                    if (CandVel_v[candId] >= v_inev || CandVel_w[candId] >= w_inev)
                    {
                        d_U = 0;
                    }
                    else
                    {
                        d_U = std::min(fabs((v_inev - CandVel_v[candId]) / v_inev), fabs((w_inev - CandVel_w[candId]) / w_inev));
                    }

                    isCollision.push_back(true);
                    // CandVel[candId].push_back(d_U);
                    DWA_var::d_U.push_back(d_U);
                    break;
                }
                else if (traj_id == traj_size - 1)
                {
                    isCollision.push_back(false);
                    d_U = 1.0;
                    //CandVel[candId].push_back(d_U);
                    DWA_var::d_U.push_back(d_U);

                }
            }
        }
    }
    else
    {
        for (int candId = 0; candId < CandVel_v.size(); candId++)
        {
            //CandVel[candId].push_back(1.0);
            d_U=1.0;
            DWA_var::d_U.push_back(d_U);
            isCollision.push_back(false);
        }
    }
    std::cout  << isCollision.size() << " " <<CandVel_v.size() <<std::endl;  
    assert(isCollision.size() == CandVel_v.size());
}

// 評価関数を計算する Dはsat係数.あらかじめ計算しておく
// 最小の評価関数の添字を返す
void my_robo::cal_J_sharedDWA(double D)
{
    double adm_min;
    double cost = 1000;
    double head_min;
    double vel_min;
    double arctan2_h;
    int index = 0;

    if (sensor.joy_cmd_vel[1] == 0 && sensor.joy_cmd_vel[0] == 0)
    {
        arctan2_h = 0;
    }
    else
    {
        arctan2_h = atan2(sensor.joy_cmd_vel[0], sensor.joy_cmd_vel[1]);
    }

    for (int i = 0; i < CandVel_v.size(); i++)
    {
        // double adm = 1 - CandVel[i][2];
        double adm = 1 - DWA_var::d_U[i];
        double head, velocity;

        head = cal_head_cost(i,arctan2_h);
        velocity = cal_vel_cost(i);

        // 最終的なコストの計算
        double temp = adm + (1 - adm) * (k_heading * head + k_velocity * velocity);

        // ROS_INFO("CandVel:%f,%f",CandVel_v[i],CandVel_w[i]);
        // ROS_INFO("joy:%f",sensor.joy_cmd_vel[0],sensor.joy_cmd_vel[1]);
        // ROS_INFO("D:%lf",D);

        if (temp < cost)
        {
            //ROS_INFO(co)
            cost = temp;
            adm_min = adm;
            head_min = head;
            vel_min = velocity;
            index = i;

            // ROS_INFO("i:%d", i);
            // ROS_INFO("now vel:%f, %f", sensor.odom.twist.twist.linear.x, sensor.odom.twist.twist.angular.z);
            // ROS_INFO("joy vel:%f, %f", sensor.joy_cmd_vel[0], sensor.joy_cmd_vel[1]);
            // ROS_INFO("cand vel:%f, %f", CandVel_v[i], CandVel_w[i]);
            // ROS_INFO("joy atan2:%f", atan2(sensor.joy_cmd_vel[0], sensor.joy_cmd_vel[1]));
            // ROS_INFO("cand atan2:%f ", atan2(CandVel_v[i],CandVel_w[i]));

            // ROS_INFO("adm:%lf", adm);
            // ROS_INFO("head:%lf", head);
            // ROS_INFO("velocity:%lf", velocity);
            // ROS_INFO("cost:%lf\n", temp);
        }
    }
    // ROS_INFO("finish cal_J");
    //ROS_INFO("index:%d\n",index);
    //ROS_INFO("pubvel:%f,%f",CandVel[index][0],CandVel[index][1]);

    // ROS_INFO("adm:%lf", a);
    // ROS_INFO("head:%lf", h);
    // ROS_INFO("velocity:%lf", v);
    // ROS_INFO("cost:%f\n", cost);
    selected.adm = adm_min;
    selected.vel_h_cost = vel_min;
    selected.head_h_cost = head_min;
    selected.cost = cost;
    opt_index = index;

    // LOG.push_back(adm_min);
    // LOG.push_back(1 - adm_min);
    // LOG.push_back(vel_min);
    // LOG.push_back(head_min);
    // LOG.push_back(cost);
    // LOG.push_back(CandVel[index][0]);
    // LOG.push_back(CandVel[index][1]);
    // LOG.push_back(sensor.joy_cmd_vel[0]);
    // LOG.push_back(sensor.joy_cmd_vel[1]);
    // LOG.push_back(sensor.odom.twist.twist.linear.x);
    // LOG.push_back(sensor.odom.twist.twist.angular.z);

    // logfile << LOG[0];
    // for (int i = 1; i < LOG.size(); i++)
    // {
    //     logfile << "," << LOG[i];
    // }
    // logfile << std::endl;
}

double my_robo::cal_head_cost(int candId,double arctan2_h)
{
    double arctan2_cand,head;

    // 候補速度のatan2の計算
    if (CandVel_v[candId] == 0 && CandVel_w[candId] == 0)
    {
        arctan2_cand = 0;
    }
    else
    {
        arctan2_cand = atan2(CandVel_v[candId], CandVel_w[candId]);
    }
    head = fabs(arctan2_cand - arctan2_h) / M_PI;

    return head;
}

double my_robo::cal_vel_cost(int trajidx)
{
    double cost = fabs(CandVel_v[trajidx] - sensor.joy_cmd_vel[0]) / spec.x_max_vel;
    if (isnan(cost))
    {
        std::cout << "isnan vel h cost" << std::endl;
        cost = 0;
    }
    //cout << "vel_h_cost: " << cost <<endl;
    return cost;
}
