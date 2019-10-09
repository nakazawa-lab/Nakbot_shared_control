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
    const double point_scale_th = 2;        // maxang/maxvel * 2 = 3.75

    // 最終的に採用する軌道のインデックス
    int opt_index;

    struct selected_vel_info{
        public:
        double linadm;
        double linsafe;
        double angadm;
        double angsafe;

        double vel,ang;
        double vel_h_cost;
        double head_h_cost;
        double cost;
    };
    selected_vel_info selected;
 
    // LRFのkd木
    kdt::KDTree<MyPoint> LRFkdtree;

    // LRFについてのPoints
    std::vector<MyPoint> LRFpoints;

    // distを計算せず、直接d thのcostを計算していくときに保持するコスト
    std::vector<std::vector<double>> dist_lin_ang;

    // DWA_var DWA;

        // treeidxのツリーのidx番目の点と、queryの点の距離(を求める。
    double cal_Dist(MyPoint query, int idx);

    double cal_head_cost_pro(int);

    double cal_vel_cost_pro(int);

    void proposed_0930();

    void cal_opt_0930();

    void kd_tree_0930();

    void record_param();

    void make_mylog(double,double,double,double,double,double,double,int);

    void make_mylog_perloop(double);

    visualization_msgs::MarkerArray make_traj_marker_array(int index);

    void plot_gnuplot(FILE *gp);

public:
    MyDWA(){
    };

    void DWAloop();

    void clear_vector();

    std::ofstream mylogfile;

    void Proposed();

    visualization_msgs::Marker make_nearest_LRF_marker(int optId);
};

#endif