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

    std::vector<double> dists;

    // d-theta平面上での距離を図る際に、スケールを合わせるために掛ける数字。
    // 例えば、thをdegで表してスケールを整えないと角度のズレに対して非常に敏感になってしまい、少しでも角度がずれていると危険と判断されてしまう。
    const double point_scale_d = 1;
    const double point_scale_th = 8;        // max_vel/max_angvel

    // 最終的に採用する軌道のインデックス
    int opt_index;
    
    // 軌道ごとのコストを保存
    //std::vector<std::array<double,2>> costs;
 
    // LRFのkd木
    kdt::KDTree<MyPoint> LRFkdtree;

    // LRFについてのPoints
    std::vector<MyPoint> LRFpoints;

    std::vector<double> costs;

    // distを計算せず、直接d thのcostを計算していくときに保持するコスト
    std::vector<std::vector<double>> dist_lin_ang;

    std::vector<double> collisionTime;

    // DWA_var DWA;

        // treeidxのツリーのidx番目の点と、queryの点の距離(を求める。
    double cal_Dist(MyPoint query, int idx);

    void cal_lin_ang_Dist(int, int,std::vector<std::vector<double>>&, double&, int );

    void cal_costs(int);

    void cal_costs_0926(int, double);

    double cal_head_cost_pro(int);

    double cal_vel_cost_pro(int);

    // costをdistを介さずけいさんするための関数。kdツリーは使わない
    void cal_dist_sep();

    void cal_cost_sep();

    double cal_lincost_sep_(int,int);

    double cal_angcost_sep_(int,int);

    void proposed_0926();

    void cal_opt_0926();

    void kd_tree_0926();

    void record_param();

    void make_mylog(double,double,double,double,double,int);

    visualization_msgs::MarkerArray make_traj_marker_array(int index);

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
    void search_LRF_Traj();

    void DWAloop();

    void clear_vector();

    std::vector<double> LOG_MYDWA;

    std::ofstream mylogfile;

    void Proposed();
};

#endif