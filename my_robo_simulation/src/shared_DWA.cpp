#include "my_robo_simulation/my_robo_drive.h"
#include "my_robo_simulation/my_robo_util.h"

using namespace std;

// 速度コストのsaturation係数を求める
double my_robo::cal_vel_sat()
{
    int num = CandVel.size();
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

// 障害物との入力相当距離(SharedDWAのd_Uにあたる)を求め,
// CandVelの3番目の要素にd_Uをプッシュバックする
// void my_robo::cal_Dist()
// {
//     double adm;
//     double t = 0.05, time;
//     double v, w, r;      // rは曲率半径
//     double d_lin, d_ang; // sharedDWAの論文における定義
//     double d, th;        // レーザセンサに対応した距離と角度
//     double d_U;

//     // <<必要な情報>>
//     // 候補速度ベクトル

//     // センサデータから計算される,障害物の位置(scan.data)

//     // <<やること>>
//     //ROS_INFO("start cal dist.");
//     int candSize = CandVel.size();
//     // 1 (v,w)を一つ取り出す
//     // 5 1から4をすべての候補に対して繰り返す
//     //   ただ、(v,w)がともに0のときは距離1となる
//     //ROS_INFO("candsize%d",candSize);
//     for (int i = 0; i < candSize; i++)
//     {
//         v = CandVel[i][0];
//         w = CandVel[i][1];
//         //ROS_INFO("i=%d",i);
//         time = 0;
//         // 4-b-1 予測時刻を増やしても衝突せず、センサ範囲を超えたら、その(v,w)でのd_uは1とする
//         if (v == 0 && w == 0)
//         {
//             d_U = 1;
//             CandVel[i].push_back(d_U);
//             //ROS_INFO("i=%d v=w=0 no collision.\n",i);
//             //ROS_INFO("i=%d d_U%f", i,d_U);
//         }
//         else
//         {
//             r = v / w;
//             // 進んだ距離(d_lin,d_ang)を計算し、dとthに変換してレーザセンサの値と比較、衝突判定する
//             // 2 予測時刻をどんどん増やしていってその時のdとthetaを求める(センサの値と比較するため)
//             //   予測時刻は衝突するのがわかればいいのでざっくり増やしていく
//             //   ↑d_linとd_angを求める必要があるので細かいほうが良い
//             while (true)
//             {
//                 time += t;

//                 if (time > PredictTime)
//                 {
//                     isCollision.push_back(false);
//                     d_U = 1;
//                     CandVel[i].push_back(d_U);
//                     // ROS_INFO("i:%d", i);
//                     // ROS_INFO("timeout while cal D_U.break");
//                     // ROS_INFO("i=%d d_U%f\n", i,d_U);
//                     break;
//                 }

//                 // d_linとd_angを求める
//                 // dとthを求める
//                 if (w == 0.0)
//                 {
//                     //ROS_INFO("w=0.0");
//                     d_lin = abs(v * time);
//                     d_ang = 0;

//                     d = d_lin;
//                     th = 0;
//                 }
//                 else
//                 {
//                     d_lin = abs(v * time);
//                     d_ang = w * time;

//                     d = sqrt(2 * r * r * (1 - cos(d_ang))); // 余弦定理
//                     if (v > 0)
//                         th = acos((v * sin(d_ang)) / (d * w)); // 0からPIまででかえされる
//                     else
//                         th = -1 * acos((v * sin(d_ang)) / (d * w));
//                 }
//                 // ROS_INFO("c2.");
//                 // ROS_INFO("d:%f.",d);
//                 // ROS_INFO("th:%f.",th);

//                 // 3 thを挟むようなLRFの計測値を2つ取り出す
//                 // indexとindex+1
//                 int index = 0;
//                 double index_th = 0;

//                 // indexが数えやすいよう、最小測定角を0度としたときの角度に直す
//                 // th_lはindexに合わせて、最小測定角から換算した角度を示す
//                 // angle_max = 2.09444 ,angle_min = -2.0944を示す = 120deg
//                 double th_l = sensor.latest_scan.angle_max + th;

//                 while (abs(index_th - th_l) > sensor.latest_scan.angle_increment)
//                 {
//                     //ROS_INFO("index:%d.",index);
//                     index++;
//                     index_th = index * sensor.latest_scan.angle_increment;
//                     //ROS_INFO("c4.");
//                     if (index > sensor.latest_scan.ranges.size() - 2)
//                     {
//                         // 120度以上の方向に行ってしまう場合
//                         // ただbreakするだけでは衝突判定が出てしまうので、特殊な数字を用意しておく
//                         index = -1;
//                         break;
//                     }
//                 }

//                 //ROS_INFO("range_size:%d",sensor.latest_scan.ranges.size());   //683

//                 //ROS_INFO("time%f",time);
//                 //ROS_INFO("index:%d",index);
//                 //ROS_INFO("d:%f",d);
//                 //ROS_INFO("laser:%f",sensor.latest_scan.ranges[index]);

//                 // 4 その2つの計測値のdの値と計算された時のdの値を求める
//                 // 4-a-1 計測値のどちらかより計算値が大きければ、衝突と判定、その時のd_lとd_aを求める
//                 if (index >= 0)
//                 {
//                     if (d > sensor.latest_scan.ranges[index] || d > sensor.latest_scan.ranges[index + 1])
//                     {
//                         //ROS_INFO("spec x_min_acc z_min_acc:%f,%f",spec.x_min_acc,spec.z_min_acc);
//                         // 4-a-2 v_inevとw_inevを求める
//                         double v_inev = sqrt(abs(2 * spec.x_min_acc * d_lin));
//                         double w_inev = sqrt(abs(2 * spec.z_min_acc * d_ang));

//                         // ROS_INFO("i=%d colision",i);
//                         // ROS_INFO("index:%d",index);
//                         // ROS_INFO("laser:%f",sensor.latest_scan.ranges[index]);
//                         // ROS_INFO("v_inev%f",v_inev);
//                         // ROS_INFO("w_inev%f",w_inev);
//                         // ROS_INFO("v%f",v);
//                         // ROS_INFO("w%f",w);
//                         // ROS_INFO("d:%f.",d);
//                         // ROS_INFO("th:%f.",th);
//                         // ROS_INFO("d_lin:%f.",d_lin);
//                         // ROS_INFO("d_ang:%f.",d_ang);
//                         // ROS_INFO("time:%f.",time);
//                         // 4-a-3 d_uを求める
//                         if (v >= v_inev || w >= w_inev)
//                         {
//                             d_U = 0;
//                             //ROS_INFO("i=%d d_U%f\n", i,d_U);
//                         }
//                         else
//                         {
//                             d_U = std::min(abs((v_inev - v) / v_inev), abs((w_inev - w) / w_inev));
//                             // ROS_INFO("d_U=%f",d_U);
//                             // ROS_INFO("i=%d, abs(v_inev- v / v_inev)%f. abs(w_inev- w / w_inev)%f\n",i, abs((v_inev - v) / v_inev),abs((w_inev- w) / w_inev));

//                             //ROS_INFO("i=%d, d_U=%f",i, d_U);ROS_INFO("d:%f.",d);
//                             //ROS_INFO("d:%f.", d);
//                             //ROS_INFO("th:%f.\n", th);
//                         }
//                         CandVel[i].push_back(d_U);

//                         //ROS_INFO("abs( w_inev - w / w_inev)%f",abs( (w_inev - w) / w_inev));
//                         //ROS_INFO("i=%d d_U%f", i,d_U);

//                         // Dの計算に使うので、もし予測時刻内の衝突であればフラグを立てる
//                         if (time < PredictTime)
//                         {
//                             //ROS_INFO("collision in predict time.");
//                             //ROS_INFO("break\n.");
//                             isCollision.push_back(true);
//                         }
//                         else
//                         {
//                             isCollision.push_back(false);
//                             //ROS_INFO("collision out of predict time.");
//                             //ROS_INFO("break\n.");
//                         }
//                         break;
//                     }
//                     else
//                     {
//                         // ループを続ける
//                         //ROS_INFO("no collision.");
//                     }
//                 }
//             }
//         }
//     }
// }

// 予め計算しておいた何度かずつのSCANの位置に従って距離を計算する
void my_robo::cal_Dist2()
{
    double adm;
    double t = 0.05, time;
    double v, w, r;      // rは曲率半径
    double d_lin, d_ang; // sharedDWAの論文における定義
    double d, th;        // レーザセンサに対応した距離と角度
    double d_U;
    double threshold = 0.3;

    // 軌道に対する繰り返し
    for (int i = 0; i < CandVel.size(); i++)
    {
        int trajsize = PredictTraj[i].size();

        std::vector<double> dist;
        std::vector<double> coltime;
        // 障害物1つずつに対する繰り返し
        for (int k = 0; k < sensor.obs.size(); k++)
        {

            // 軌道時刻1つずつに対する繰り返し
            for (int j = 0; j < trajsize; j++)
            {
                double dist_temp = cal_euclid(PredictTraj[i][j][1], PredictTraj[i][j][2], sensor.obs[k][0], sensor.obs[k][1]);

                // 衝突したら、そのときの距離と時刻を保存しておく
                if (dist_temp < threshold)
                {
                    dist.push_back(dist_temp);
                    coltime.push_back(PredictTraj[i][j][0]);
                    // 軌道時刻に対するループを抜ける
                    break;
                }
            }
        }

        // 衝突なし
        if (dist.size() == 0)
        {
            d_U = 1.0;
            CandVel[i].push_back(d_U);
            isCollision.push_back(false);

            std::cout << "i:" << i << " candvel: (" << CandVel[i][0] << ", " << CandVel[i][1] << ")" << std::endl;
            std::cout << "coltime: nan" << std::endl;
            std::cout << "d_U:" << d_U << std::endl
                      << std::endl;
        }
        //　衝突あり 複数ある場合は最大となるときのｄ＿U を求める
        else
        {
            int mindistIndex = std::min_element(dist.begin(), dist.end()) - dist.begin();
            d_lin = CandVel[i][0] * coltime[mindistIndex];
            d_ang = CandVel[i][1] * coltime[mindistIndex];

            double v_inev = sqrt(abs(2 * spec.x_min_acc * d_lin));
            double w_inev = sqrt(abs(2 * spec.z_min_acc * d_ang));

            if (CandVel[i][0] >= v_inev || CandVel[i][1] >= w_inev)
            {
                d_U = 0;
            }
            else
            {
                d_U = std::min(abs((v_inev - CandVel[i][0]) / v_inev), abs((w_inev - CandVel[i][1]) / w_inev));
            }

            isCollision.push_back(true);
            CandVel[i].push_back(d_U);

            std::cout << "i:" << i << " candvel: (" << CandVel[i][0] << ", " << CandVel[i][1] << ")" << std::endl;
            std::cout << "coltime:" << coltime[mindistIndex] << std::endl;
            std::cout << "d_U:" << d_U << std::endl
                      << std::endl;
        }
    }
}

// 評価関数を計算する Dはsat係数.あらかじめ計算しておく
// 最小の評価関数の添字を返す
int my_robo::cal_J_sharedDWA(double D)
{
    double adm_min;
    double cost = 1000;
    double head_min;
    double vel_min;
    int index = 0;

    for (int i = 0; i < CandVel.size(); i++)
    {
        double adm = 1 - CandVel[i][2];
        double head, velocity;

        head = cal_head_cost(i);
        velocity = cal_vel_cost(i);

        // 最終的なコストの計算
        double temp = adm + (1 - adm) * (k_heading * head + k_velocity * velocity);

        // ROS_INFO("CandVel:%f,%f",CandVel[i][0],CandVel[i][1]);
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
            // ROS_INFO("cand vel:%f, %f", CandVel[i][0], CandVel[i][1]);
            // ROS_INFO("joy atan2:%f", atan2(sensor.joy_cmd_vel[0], sensor.joy_cmd_vel[1]));
            // ROS_INFO("cand atan2:%f ", atan2(CandVel[i][0],CandVel[i][1]));

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
    LOG.push_back(adm_min);
    LOG.push_back(1 - adm_min);
    LOG.push_back(vel_min);
    LOG.push_back(head_min);
    LOG.push_back(cost);
    LOG.push_back(CandVel[index][0]);
    LOG.push_back(CandVel[index][1]);
    LOG.push_back(sensor.joy_cmd_vel[0]);
    LOG.push_back(sensor.joy_cmd_vel[1]);

    logfile << LOG[0];
    for (int i = 1; i < LOG.size(); i++)
    {
        logfile << "," << LOG[i];
    }
    logfile << std::endl;

    return index;
}

double my_robo::cal_head_cost(int candId)
{
    double arctan2_cand, arctan2_h, head;

    // 候補速度のatan2の計算
    if (CandVel[candId][0] == 0 && CandVel[candId][1] == 0)
    {
        arctan2_cand = 0;
    }
    else
    {
        arctan2_cand = atan2(CandVel[candId][0], CandVel[candId][1]);
    }

    if (sensor.joy_cmd_vel[1] == 0 && sensor.joy_cmd_vel[0] == 0)
    {
        arctan2_h = 0;
    }
    else
    {
        arctan2_h = atan2(sensor.joy_cmd_vel[0], sensor.joy_cmd_vel[1]);
    }

    head = abs(arctan2_cand - arctan2_h) / M_PI;

    // //　joyの速度指令に応じて, コストを求める
    // // 2 速度指令が共に０のとき
    // if (sensor.joy_cmd_vel[0] == 0 && sensor.joy_cmd_vel[1] == 0)
    // {
    //     //ROS_INFO("both 0");
    //     head = abs(CandVel[candId][1]) / M_PI;
    // }
    // // 1 角速度指令が0、速度指令値が0ではないとき
    // else if (sensor.joy_cmd_vel[0] != 0 && sensor.joy_cmd_vel[1] == 0)
    // {
    //     //          ROS_INFO("only w 0");
    //     if(CandVel[candId][0]==0 && CandVel[candId][1]==0) head = M_PI;
    //     else head = abs(M_PI/2 - arctan2) / M_PI;
    // }
    // // 3 角速度指令が0ではなく、速度指令が0のとき
    // else if (sensor.joy_cmd_vel[0] == 0 && sensor.joy_cmd_vel[1] != 0)
    // {
    //     head = abs((CandVel[candId][1] - sensor.joy_cmd_vel[1])) / M_PI;
    // }
    // else
    // {
    //     head = abs(arctan2 - atan2(sensor.joy_cmd_vel[0], sensor.joy_cmd_vel[1])) / M_PI;
    // }

    return head;
}

double my_robo::cal_vel_cost(int trajidx)
{
    double cost = abs(CandVel[trajidx][0] - sensor.joy_cmd_vel[0]) / spec.x_max_vel;
    if (isnan(cost))
    {
        std::cout << "isnan vel h cost" << std::endl;
        cost = 0;
    }
    //cout << "vel_h_cost: " << cost <<endl;
    return cost;
}
