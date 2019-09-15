#include "ros/ros.h"
#include <vector>


#ifndef MY_VAR
#define MY_VAR

// 次の位置を格納する箱
struct position
{
    double x = 0;
    double y = 0;
    double sin_th = 0;
    double cos_th = 1;
};


// DWAのセッティング
struct DWA_var
{
    // DWA設定の刻み.ループレイトと同じが望ましい
    double dt = 0.5;
    // 軌道計算の刻み
    double dt_traj = 0.2;
    // 軌道予測時刻
    double PredictTime = 5;
    //double looprate = 2;          // Hz
    double looprate = 1 / dt;

    double k_heading = 1;
    double k_velocity = 1;

    // この秒数後の衝突に対して衝突危険正規化距離を１未満にする。これ以上の場合は1で安全
    float thes_vel = 2;
    float thes_ang = 2;

    // 予測軌道 [index][時刻index][time,x,y,sin cos]
    std::vector<std::vector<std::vector<double>>> PredictTraj;

    // 予測軌道の相対位置 [index][time index][time,d,theta]
    std::vector<std::vector<std::vector<double>>> PredictTraj_r;

    std::vector<std::vector<double>> Joy_PredictTraj;

    // 候補となる(v,w)の対
    // [index][v,w,d_U]
    std::vector<std::vector<double>> CandVel;

    // 衝突判定用フラグ
    std::vector<bool> isCollision;
};

#endif