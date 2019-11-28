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
class DWA_var
{
public:
    // DWA設定の刻み.ループレイトと同じが望ましい
    const double dt = 0.3;
    // 軌道計算の刻み
    const double dt_traj = 0.1;
    // 軌道予測時刻
    const double PredictTime = 3.0;
    const double looprate = 10.0;

    const double k_heading = 1.0;
    const double k_velocity = 1.0;

    // この秒数後の衝突に対して衝突危険正規化距離を１未満にする。これ以上の場合は1で安全
    const float thres_vel_time = 3.0;
    const float thres_ang_time = 3.0;

    const int POINT_INTERVAL = 2;

    //const double DWA_RESOLUTION_DIV = 5;    //(0.8,0)のとき67候補点, (0.8,1.5)のとき31候補点くらい
    const double DWA_RESOLUTION_DIV = 10;

    const int LINSAFE_MULTIPLIER = 2;
    const int ANGSAFE_MULTIPLIER = 2;
    const int PUB_TRAJ_MARKER_PER_LOOP =3;

    const bool IsREAL = false;

    std::chrono::time_point<std::chrono::_V2::system_clock, std::chrono::nanoseconds> start_time;

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
