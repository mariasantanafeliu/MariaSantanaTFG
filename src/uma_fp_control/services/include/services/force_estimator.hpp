#pragma once

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Trigger.h>
#include <mutex>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <Eigen/Dense>
#include <string>
#include <vector>
#include <array>
#include <fstream>
#include <sstream>

using Matrix4d  = Eigen::Matrix4d;
using Vector3d  = Eigen::Vector3d;
using Vector5d  = Eigen::Matrix<double,5,1>;
using Matrix5d  = Eigen::Matrix<double,5,5>;

// Sincronización 
/*using SyncPolicy3 = message_filters::sync_policies::ApproximateTime<
    std_msgs::Float64MultiArray,   
    std_msgs::Float64MultiArray,   
    std_msgs::Float64MultiArray>;  */

// Filtro 
struct IIRFilter {
    std::vector<double> b, a;   // coeficientes (a[0] == 1 normalizado)
    std::vector<std::vector<double>> z;  // estados internos (orden x canales)
    int order;
    int channels;

    void init(const std::vector<double>& b_in,
              const std::vector<double>& a_in,
              int n_channels)
    {
        b = b_in; a = a_in;
        order    = (int)b.size() - 1;
        channels = n_channels;
        z.assign(order, std::vector<double>(channels, 0.0));
    }

    // Filtra un vector (1 muestra, n canales) en su lugar
    void filter(std::vector<double>& x)
    {
        for (int c = 0; c < channels; ++c) {
            double xn = x[c];
            double yn = b[0]*xn + z[0][c];
            for (int i = 0; i < order-1; ++i)
                z[i][c] = b[i+1]*xn - a[i+1]*yn + z[i+1][c];
            z[order-1][c] = b[order]*xn - a[order]*yn;
            x[c] = yn;
        }
    }

    void reset() {
        for (auto& row : z)
            std::fill(row.begin(), row.end(), 0.0);
    }
};

enum class Mode { CALIBRATING, FROZEN, SURGERY };

class ForceEstimator
{
public:
    explicit ForceEstimator(ros::NodeHandle& nh, ros::NodeHandle& nh_priv);

private:
    // Callbacks 
    void poseCb(const std_msgs::Float64MultiArray::ConstPtr& pose_msg);
    void robotForceCb(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void tissueForceCb(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void abdomenCb(const std_msgs::Float64MultiArray::ConstPtr& msg);
    void velocityCb(const std_msgs::Float64MultiArray::ConstPtr& msg);

    // Servicios
    bool srvTare(std_srvs::Trigger::Request&,  std_srvs::Trigger::Response& res);
    bool srvFreeze(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res);

    Matrix4d loadCSV(const std::string& path);
    void     initButterworth();
    void     initTheta();
    void     processOneSample(const Vector3d& pos_world,
                              const Vector3d& F_robot,
                              const Vector3d& F_tissue,
                              const Vector3d& F_abdomen,
                              const std::vector<double>& V_robot);

    // RLS para un eje
    void rlsUpdate(Vector5d& th, Matrix5d& P,
                   const Vector5d& phi, double error, double lambda);

    // ── Parámetros (cargados de ROS param / hardcoded) ────────────────────────
    double Ts_          = 1.0/125.0;
    double Z_sup_       = 0.0605;    // superficie tejido (world2tissue)
    double Z_adh_       = 0.003;
    int    RAMP_WIN_    = 10;
    double lambda_rls_  = 0.999;
    double tol_sat_     = 0.2;       // saturación superior Fp_tej

    // Parámetros boost hiperelasticidad
    double boost_X_gain_  = 2.0,  boost_X_scale_ = 0.003;
    double boost_Y_gain_  = 3.5,  boost_Y_scale_ = 0.003;

    // Parámetros trinquete
    double fraw_min_  = 0.25,  fraw_max_  = 0.8,  fraw_kvel_ = 80.0;

    // Matrices de transformación 
    // T12 = auto2darel    T23 = darel2world (= inv(world2darel))
    // T13 = T12 * T23     R_Mesa_Auto = T13.R'
    Matrix4d T12_, T23_, T13_;
    Eigen::Matrix3d R_Mesa_Auto_;   // transforma F_total → frame mundo
    Eigen::Matrix3d R_Darel_Mesa_;  // transforma F_abdomen → frame mundo

    // Filtros 
    IIRFilter filt_robot_;    
    IIRFilter filt_tissue_;   
    IIRFilter filt_abdomen_;  
    IIRFilter filt_pos_;      

    // Taraje 
    bool tare_requested_  = false;
    bool tared_           = false;
    // Acumuladores para taraje 
    static constexpr int N_TARE_ = 50;
    int  tare_count_      = 0;
    std::vector<double> tare_robot_acc_   = {0,0,0};
    std::vector<double> tare_tissue_acc_  = {0,0,0};
    std::vector<double> tare_abdomen_acc_ = {0,0,0};
    Vector3d offset_robot_   = Vector3d::Zero();
    Vector3d offset_tissue_  = Vector3d::Zero();
    Vector3d offset_abdomen_ = Vector3d::Zero();

    // Modo 
    Mode mode_ = Mode::CALIBRATING;

    // Theta y covarianzas RLS 
    Vector5d thX_, thY_, thZ_;
    Matrix5d PX_,  PY_,  PZ_;

    // Theta del modelo offline
    Vector5d thX_off_, thY_off_, thZ_off_;

    // Estado interno del bucle ARX 
    bool    en_cont_  = false;
    double  pto_X_    = 0, pto_Y_ = 0, prof_mx_ = 0;
    int     contact_k_= 0;
    double  fac_cur_  = 0.8;
    std::array<double,3> xk_     = {0,0,0};
    std::array<double,3> xe_     = {0,0,0};
    std::array<double,3> xk_1_   = {0,0,0};
    std::array<double,3> xk_2_   = {0,0,0};
    std::array<double,3> Fk_1_   = {0,0,0};
    std::array<double,3> Fk_2_   = {0,0,0};

    std::array<double,3> Fk_1_off_   = {0,0,0};
    std::array<double,3> Fk_2_off_   = {0,0,0};

    // Última posición filtrada (para detección de movimiento en taraje)
    std::array<double,3> pos_filt_prev_ = {0,0,0};

    // Última fuerza recibida (callback separado)
    std::mutex data_mutex_;
    std::vector<double> latest_robot_force_ = {0,0,0};
    std::vector<double> latest_tissue_force_ = {0,0,0};
    Vector3d latest_abdomen_force_ = Vector3d::Zero();
    std::vector<double> latest_velocity_ = {0,0,0};

    // Publishers 
    ros::Publisher pub_tej_est_;      // estimación tejido
    ros::Publisher pub_abd_est_;      // estimación abdomen
    ros::Publisher pub_robot_tared_;  // F_total tarada (para PlotJuggler)
    ros::Publisher pub_tej_real_;     // F_tejido real tarada
    ros::Publisher pub_abd_real_;     // F_abdomen real tarada
    ros::Publisher pub_tej_off_;      // F_tejido del modelo offline
    ros::Publisher pub_abd_off_;      // F_abdomen del modelo offline

    // Subscribers 
    ros::Subscriber sub_pose_;
    ros::Subscriber sub_robot_force_;
    ros::Subscriber sub_tissue_force_;
    ros::Subscriber sub_abdomen_;
    ros::Subscriber sub_velocity_;

    // Servicios 
    ros::ServiceServer srv_tare_;
    ros::ServiceServer srv_freeze_;

    std::string config_path_;
};
