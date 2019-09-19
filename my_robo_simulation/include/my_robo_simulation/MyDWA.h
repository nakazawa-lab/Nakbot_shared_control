#include "my_robo_simulation/kdtree.h"
#include "my_robo_simulation/my_robo_drive.h"

#ifndef MY_DWA
#define MY_DWA

// kdtree.hの利用のための、点を表すクラス
class MyPoint: public std::array<double,2>
{
public:
    static const int DIM =2;

    MyPoint(){}
    MyPoint(double x, double y){
        (*this)[0] = x;
        (*this)[1] = y;
    }
};

class MyDWA: public my_robo
{
private:
    std::vector<int> scan_indices, traj_indices;

    std::vector<double> lin_normdists, ang_normdists;

    // d-theta平面上での距離を図る際に、スケールを合わせるために掛ける数字。
    // 例えば、thをdegで表してスケールを整えないと角度のズレに対して非常に敏感になってしまい、少しでも角度がずれていると危険と判断されてしまう。
    const double point_scale_d = 1;
    const double point_scale_th = 0.5333333;        // max_vel/max_angvel

    // 最終的に採用する軌道のインデックス
    int opt_index;
    
    // 軌道ごとのコストを保存
    //std::vector<std::array<double,2>> costs;
 
    // LRFのkd木
    kdt::KDTree<MyPoint> LRFkdtree;

    // LRFについてのPoints
    std::vector<MyPoint> LRFpoints;

    // DWA_var DWA;

        // treeidxのツリーのidx番目の点と、queryの点の距離(を求める。
    double cal_Dist(MyPoint query, int idx);

    void cal_lin_ang_Dist(int, int,std::vector<std::vector<double>>&, double&, int );

        // 同名の関数内で呼び出す、単一の軌道に対してLRFとの再接近点を求める
    void search_LRF_Traj(sensor_msgs::LaserScan& latest_scan, std::vector<std::vector<double>>& PredictTraj);

    // kd木によって、障害物を示す点群の中から、任意の点(d,theta)に対して最短距離となる点のインデックスを求める
    // scan_index, traj_indexを求める
    int kd_tree_nnSearch(const MyPoint query);

        // LRFのスキャン点から、kdtreeを構築する
    void kd_tree(sensor_msgs::LaserScan& scan,std::vector<std::vector<std::vector<double>>>& PredictTrajs,double& robot_rad);

    void cal_costs();

public:
    MyDWA(){
    };

    MyDWA(DWA_var DWA_){
    };

    // 最短距離となる2つの点がわかっている状態で、lin_normDistとang_normDistを求める
    void cal_Dist();

    // DWAのパラメータをコピーする関数の予定。ポインタを用いて無駄なメモリを消費しないようにしたい
    void set_param(DWA_var DWA){

    };

    // 現在わかっているLRFの情報と、軌道の情報から、2つの点群が最も近いときの距離を計算、最適な候補軌道のインデックスを計算して保持しておく関数
    void search_LRF_Traj(sensor_msgs::LaserScan& latest_scan, std::vector<std::vector<std::vector<double>>>& PredictTrajs,double robot_rad);

    void clear(){
        LRFpoints.clear();
        LRFkdtree.clear();
        scan_indices.clear();
        traj_indices.clear();
        lin_normdists.clear();
        ang_normdists.clear();
    };

    void DWAloop();

    void clear_vector();
};

#endif