#include <services/force_estimator.hpp>
#include <mutex>
#include <cmath>

//  Coeficientes Butterworth orden 4, fc=3 Hz, fs=125 Hz  (Tustin / bilineal)
//  Calculados con scipy.signal.butter(4, 3/62.5, 'low') en Python
static const std::vector<double> B_LP = {
    3.37180478e-04, 1.34872191e-03, 2.02308287e-03,
    1.34872191e-03, 3.37180478e-04
};
static const std::vector<double> A_LP = {
    1.0,           -3.14515479,    3.72146949,
   -1.96750921,    0.39186971
};


//  Constructor
ForceEstimator::ForceEstimator(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
{
    // Parámetros ROS 
    nh_priv.param<std::string>("config_path", config_path_,
        ros::package::getPath("uma_fp_control") + "/config");
    nh_priv.param<double>("Z_sup",      Z_sup_,      0.0605);
    nh_priv.param<double>("Z_adh",      Z_adh_,      0.003);
    nh_priv.param<int>   ("RAMP_WIN",   RAMP_WIN_,   10);
    nh_priv.param<double>("lambda_rls", lambda_rls_, 0.999);

    // Cargar matrices de transformación 
    // T12: auto → darel
    T12_ = loadCSV(config_path_ + "/auto2darel.csv");
    // T23: darel → world
    Matrix4d T23_ = loadCSV(config_path_ + "/darel2world.csv");
    // Z_sup viene de world2tissue (posición Z de la superficie del tejido)
    Matrix4d world2tissue = loadCSV(config_path_ + "/world2tissue.csv");
    Z_sup_ = world2tissue(2,3);   // componente Z de traslación

    T13_         = T12_ * T23_;
    T13_(0,3)   += 0.0273;       // offset hardcodeado igual que en MATLAB
    R_Mesa_Auto_ = T13_.topLeftCorner<3,3>().transpose();
    R_Darel_Mesa_= T23_.topLeftCorner<3,3>();

    ROS_INFO("[ForceEstimator] Matrices de transformacion cargadas.");
    ROS_INFO_STREAM("T13 =\n" << T13_);

    // Filtros 
    filt_robot_.init   (B_LP, A_LP, 3);
    filt_tissue_.init  (B_LP, A_LP, 3);
    filt_abdomen_.init (B_LP, A_LP, 3);
    filt_pos_.init     (B_LP, A_LP, 3);

    // Theta inicial 
    initTheta();

    // Publishers 
    pub_tej_est_    = nh.advertise<std_msgs::Float64MultiArray>("tissue_force_estimated", 100);
    pub_abd_est_    = nh.advertise<std_msgs::Float64MultiArray>("abdomen_force_estimated", 100);
    pub_robot_tared_= nh.advertise<std_msgs::Float64MultiArray>("robot_force_tared",     100);
    pub_tej_real_   = nh.advertise<std_msgs::Float64MultiArray>("tissue_force_real",      100);
    pub_abd_real_   = nh.advertise<std_msgs::Float64MultiArray>("abdomen_force_real",     100);
    pub_tej_off_   = nh.advertise<std_msgs::Float64MultiArray>("tissue_force_offline",      100);
    pub_abd_off_   = nh.advertise<std_msgs::Float64MultiArray>("abdomen_force_offline",     100);


    // Servicios 
    srv_tare_   = nh.advertiseService("tare_forces",   &ForceEstimator::srvTare,   this);
    srv_freeze_ = nh.advertiseService("freeze_theta",  &ForceEstimator::srvFreeze, this);

    // Subscripciones sincronizadas 
    sub_robot_force_ = nh.subscribe("/auto/robotbase_force", 1,
                                &ForceEstimator::robotForceCb, this);
    sub_tissue_force_ = nh.subscribe("/auto/tfg/tissue_force", 1,
                                &ForceEstimator::tissueForceCb, this);
    sub_pose_ = nh.subscribe("/auto/pose_topic", 1,
                                &ForceEstimator::poseCb, this);
    sub_abdomen_ = nh.subscribe("/darel/abdomen_force_topic", 1,
                                &ForceEstimator::abdomenCb, this);
    sub_velocity_ = nh.subscribe("/auto/velocity_topic", 1, 
                                &ForceEstimator::velocityCb, this); 

    ROS_INFO("[ForceEstimator] Nodo listo. Modo: CALIBRATING");
    ROS_INFO("[ForceEstimator] Llama a /tare_forces para tarar y luego a /freeze_theta para congelar theta.");
}


//  Carga de CSV  (4×4)
Matrix4d ForceEstimator::loadCSV(const std::string& path)
{
    Matrix4d M = Matrix4d::Identity();
    std::ifstream f(path);
    if (!f.is_open()) {
        ROS_ERROR_STREAM("[ForceEstimator] No se puede abrir: " << path);
        return M;
    }
    int row = 0;
    std::string line;
    while (std::getline(f, line) && row < 4) {
        std::stringstream ss(line);
        std::string token;
        int col = 0;
        while (std::getline(ss, token, ',') && col < 4)
            M(row, col++) = std::stod(token);
        ++row;
    }
    return M;
}


//  Theta inicial (convertidos de continuo a discreto, Tustin, igual que MATLAB)
//  sys_Z_c = -tf([4050,1384],[1,1.61,0.59])
//  sys_X_c = sys_Y_c = -tf([480,590],[1,1.61,1.40])
//  Los coeficientes discretos están precalculados aquí para evitar dependencia
//  de control_toolbox en ROS.

void ForceEstimator::initTheta()
{
    // Ts = 0.008 s  →  Tustin: s = 2/Ts * (z-1)/(z+1)
    // Resultados precalculados (idénticos a c2d en MATLAB con 'tustin'):
    //
    //  Eje Z:  den=[1, -1.9936,  0.9937]   num=[-31.6816, 0.0000, 31.6816]
    //  Eje X/Y:den=[1, -1.9887,  0.9888]   num=[-3.7441,  0.0000,  3.7441]
    //
    // theta = [a1; a2; b0; b1; b2]

    // Z
    thZ_ << -1.98716,  0.98720,
            -16.1182,  -0.0440,  16.0742;
    PZ_ = 10.0 * Matrix5d::Identity();

    // X
    thX_ << -1.98711,  0.98720,
            -1.9170,   -0.0188,  1.8983;
    PX_ = 10.0 * Matrix5d::Identity();

    // Y (igual que X)
    thY_ = thX_;
    PY_  = PX_;

    //modelo offline
    thX_off_ = thX_;
    thY_off_ = thY_;
    thZ_off_ = thZ_;
}


//  Callbacks
void ForceEstimator::robotForceCb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 3) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_robot_force_ = {msg->data[0], msg->data[1], msg->data[2]};
}

void ForceEstimator::tissueForceCb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 3) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_tissue_force_ = {msg->data[0], msg->data[1], msg->data[2]};
}

void ForceEstimator::abdomenCb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    if (msg->data.size() < 3) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_abdomen_force_ << msg->data[0], msg->data[1], msg->data[2];
}

void ForceEstimator::velocityCb(const std_msgs::Float64MultiArray::ConstPtr& msg) 
{
    if (msg->data.size() < 3) return;
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_velocity_ = {msg->data[0], msg->data[1], msg->data[2]}; 
}


//  Servicio: Taraje
bool ForceEstimator::srvTare(std_srvs::Trigger::Request&,
                              std_srvs::Trigger::Response& res)
{
    // Reset acumuladores
    tare_count_ = 0;
    tare_robot_acc_   = {0,0,0};
    tare_tissue_acc_  = {0,0,0};
    tare_abdomen_acc_ = {0,0,0};
    tare_requested_   = true;
    tared_            = false;

    // Reset filtros para que arranquen limpios tras el taraje
    filt_robot_.reset();
    filt_tissue_.reset();
    filt_abdomen_.reset();
    filt_pos_.reset();

    // Reset estado ARX
    en_cont_   = false;
    contact_k_ = 0;
    fac_cur_   = 0.8;
    xk_   = {0,0,0};  xk_1_ = {0,0,0};  xk_2_ = {0,0,0};
    Fk_1_ = {0,0,0};  Fk_2_ = {0,0,0};
    Fk_1_off_ = {0,0,0};  Fk_2_off_ = {0,0,0};

    res.success = true;
    res.message = "Taraje iniciado - promediando " + std::to_string(N_TARE_) + " muestras.";
    ROS_INFO_STREAM("[ForceEstimator] " << res.message);
    return true;
}


//  Servicio: Freeze theta (calibración → cirugía)
bool ForceEstimator::srvFreeze(std_srvs::Trigger::Request&,
                                std_srvs::Trigger::Response& res)
{
    if (!tared_) {
        res.success = false;
        res.message = "ERROR: haz el taraje primero antes de congelar theta.";
        ROS_WARN_STREAM("[ForceEstimator] " << res.message);
        return true;
    }
    mode_ = Mode::FROZEN;
    res.success = true;
    res.message = "Theta congelado. Modo SURGERY activado — RLS desactivado.";
    ROS_INFO_STREAM("[ForceEstimator] " << res.message);
    return true;
}


//  Callback pose
void ForceEstimator::poseCb(const std_msgs::Float64MultiArray::ConstPtr& pose_msg)
{
    if (pose_msg->data.size() < 16) return;

    // 1. Extraer posición TCP del world desde T_TTP (column-major, 16 vals) 
    // pose_topic publica T_TTP columna por columna igual que en movimientoTFG.cpp
    Matrix4d T_TTP;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            T_TTP(j,i) = pose_msg->data[i*4+j];

    std::vector<double> pos_raw = {T_TTP(0,3), T_TTP(1,3), T_TTP(2,3)};

    // 2. Señales de fuerza
    std::vector<double> F_robot_raw, F_tissue_raw, V_robot_raw;
    Vector3d F_abdomen_raw;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        F_robot_raw = latest_robot_force_;
        F_tissue_raw = latest_tissue_force_;
        F_abdomen_raw = latest_abdomen_force_;
        V_robot_raw = latest_velocity_;
    }
    std::vector<double> F_abd_raw_v = {F_abdomen_raw(0),
                                       F_abdomen_raw(1),
                                       F_abdomen_raw(2)};

    // 3. Filtrado causal 
    //filt_pos_.filter    (pos_raw);
    filt_robot_.filter  (F_robot_raw);
    filt_tissue_.filter (F_tissue_raw);
    filt_abdomen_.filter(F_abd_raw_v);

    // Inversión de signo en Z (igual que en MATLAB)
    F_tissue_raw[2] = -F_tissue_raw[2];
    F_abd_raw_v [2] = -F_abd_raw_v[2];
    F_robot_raw [2] = -F_robot_raw[2];

    // 4. Taraje 
    if (tare_requested_ && !tared_) {
        if (tare_count_ < N_TARE_) {
            for (int i = 0; i < 3; ++i) {
                tare_robot_acc_  [i] += F_robot_raw  [i];
                tare_tissue_acc_ [i] += F_tissue_raw [i];
                tare_abdomen_acc_[i] += F_abd_raw_v  [i];
            }
            ++tare_count_;
        } else {
            for (int i = 0; i < 3; ++i) {
                offset_robot_  (i) = tare_robot_acc_  [i] / N_TARE_;
                offset_tissue_ (i) = tare_tissue_acc_ [i] / N_TARE_;
                offset_abdomen_(i) = tare_abdomen_acc_[i] / N_TARE_;
            }
            tared_          = true;
            tare_requested_ = false;
            ROS_INFO("[ForceEstimator] Taraje completado.");
        }
        return;   // no procesar hasta completar taraje
    }

    if (!tared_) return;   // esperar taraje antes de estimar

    // 5. Aplicar offsets de taraje 
    Vector3d F_robot  (F_robot_raw[0]   - offset_robot_(0),
                       F_robot_raw[1]   - offset_robot_(1),
                       F_robot_raw[2]   - offset_robot_(2));
    Vector3d F_tissue (F_tissue_raw[0]  - offset_tissue_(0),
                       F_tissue_raw[1]  - offset_tissue_(1),
                       F_tissue_raw[2]  - offset_tissue_(2));
    Vector3d F_abdomen(F_abd_raw_v[0]   - offset_abdomen_(0),
                       F_abd_raw_v[1]   - offset_abdomen_(1),
                       F_abd_raw_v[2]   - offset_abdomen_(2));

    // Abdomen al frame mundo (igual que en MATLAB: R_Darel_Mesa * Fa)
    Vector3d F_abd_world = R_Darel_Mesa_ * F_abdomen;

    // 6. Procesar muestra (ARX + RLS) 
    Vector3d pos_w(pos_raw[0], pos_raw[1], pos_raw[2]);
    processOneSample(pos_w, F_robot, F_tissue, F_abd_world, V_robot_raw);
}


//  Núcleo del algoritmo — equivalente al bucle k de MATLAB
void ForceEstimator::processOneSample(const Vector3d& pos_world,
                                       const Vector3d& F_robot,
                                       const Vector3d& F_tissue,
                                       const Vector3d& F_abdomen,
                                       const std::vector<double>& V_robot)
{
    // A. Deformación del tejido (frame mesa = frame world aquí) 
    // Transformar posición al frame mesa
    Eigen::Vector4d pw_h; pw_h << pos_world, 1.0;
    Eigen::Vector4d pm_h = T13_.inverse() * pw_h;
    double Pm_x = pm_h(0), Pm_y = pm_h(1), Pm_z = pm_h(2);
    double dZ_k = Pm_z - Z_sup_;
    ROS_INFO_THROTTLE(1.0, "Z_auto_recib: %.4f | Z_Robot_Mesa: %.4f | Z_Superficie: %.4f | Deformacion dZ_k: %.4f", pos_world(2), Pm_z, Z_sup_, dZ_k);
    
    if (dZ_k <= 0.0) {
        // Contacto con tejido
        if (!en_cont_) {
            pto_X_ = Pm_x; pto_Y_ = Pm_y;
            en_cont_ = true; prof_mx_ = dZ_k;
        }
        double dXb = Pm_x - pto_X_;
        double dYb = Pm_y - pto_Y_;
        if (dZ_k < prof_mx_) prof_mx_ = dZ_k;

        if (dZ_k > prof_mx_ + 1e-4) {
            // Subiendo — recuperación con adherencia
            double r_rec = Z_adh_ - prof_mx_;
            double rf = std::pow(
                std::max(0.0, std::min(1.0,
                    1.0 - (dZ_k - prof_mx_) / r_rec)), 1.3);
            xk_ = {dXb*rf, dYb*rf, dZ_k};
        } else {
            // Bajando o plano
            xk_ = {dXb, dYb, dZ_k};
        }
    } else if (dZ_k > 0.0 && dZ_k < Z_adh_ && en_cont_) {
        // Zona de adherencia (saliendo)
        double r_rec = Z_adh_ - prof_mx_;
        double rf = std::pow(
            std::max(0.0, std::min(1.0,
                1.0 - (dZ_k - prof_mx_) / r_rec)), 1.3);
        xk_ = {(Pm_x - pto_X_)*rf,
                (Pm_y - pto_Y_)*rf,
                dZ_k * (1.0 - dZ_k / Z_adh_)};
    } else {
        // Sin contacto — reset
        en_cont_ = false;
        pto_X_ = Pm_x; pto_Y_ = Pm_y; prof_mx_ = 0.0;
        xk_   = {0,0,0};
        Fk_1_ = {0,0,0}; Fk_2_ = {0,0,0};
    }

    // B. Contador de contacto y rampa 
    if (dZ_k <= 0.0 && en_cont_)
        contact_k_ = std::min(contact_k_ + 1, RAMP_WIN_);
    else if (dZ_k > 0.0 && dZ_k < Z_adh_ && en_cont_)
        contact_k_ = std::max(contact_k_ - 1, 0);
    else
        contact_k_ = 0;

    double ramp = static_cast<double>(contact_k_) / RAMP_WIN_;

    // C. Boost hiperelasticidad 
    std::array<double,3> xe;
    xe[0] = xk_[0] * (1.0 + boost_X_gain_ * std::exp(-std::abs(xk_[0]) / boost_X_scale_));
    xe[1] = xk_[1] * (1.0 + boost_Y_gain_ * std::exp(-std::abs(xk_[1]) / boost_Y_scale_));
    xe[2] = xk_[2];
    xe[0] *= ramp; xe[1] *= ramp; //xe[2] *= ramp;

    // D. Velocidad y trinquete 
    // Velocidad calculada por el histórico de deformación
    double fs = 1.0 / Ts_;
    double vxe0 = (xe[0] - xk_1_[0]) * fs;
    double vxe1 = (xe[1] - xk_1_[1]) * fs;
    double vxe2 = (xe[2] - xk_1_[2]) * fs;
    double vel_lat_calc = std::sqrt(vxe0*vxe0 + vxe1*vxe1);

    // Velocidad del topic (rotarla para pasarla al frame mesa)
    Eigen::Vector3d V_mesa = R_Mesa_Auto_ * Eigen::Vector3d(V_robot[0], V_robot[1], V_robot[2]);
    double vel_lat_topic = std::sqrt(V_mesa(0)*V_mesa(0) + V_mesa(1)*V_mesa(1));

    // Si el topic viene vacío/cero pero nos estamos moviendo, usamos la calculada
    double vel_lat = vel_lat_topic;
    if (std::abs(vel_lat_topic) < 1e-4 && vel_lat_calc > 1e-3) {
        vel_lat = vel_lat_calc;
    }


    //ROS_INFO_THROTTLE(1, "[VEL] V topic: %.4f, | V calc: %.4f", V_mesa(2), vxe2);

    double fraw = std::max(fraw_min_, fraw_max_ - vel_lat * fraw_kvel_);
    if (dZ_k < 0.0 && fraw < fac_cur_)
        fac_cur_ = fraw;
    else if (dZ_k >= 0.0)
        fac_cur_ = fraw_max_;

    xe[0] *= fac_cur_;
    xe[1] *= fac_cur_;

    // E. Warm-start en activación AR 
    double ar = (contact_k_ >= RAMP_WIN_) ? 1.0 : 0.0;
    static bool warm_start_done = false;

    if (contact_k_ == RAMP_WIN_ && !warm_start_done) {
        double E0_XY = -590.0 / 1.40;
        double E0_Z  = -1384.0 / 0.59;
        Fk_1_ = {E0_XY*xe[0], E0_XY*xe[1], E0_Z*xe[2]};
        Fk_2_ = Fk_1_;
        Fk_1_off_ = Fk_1_;
        Fk_2_off_ = Fk_1_;
        warm_start_done = true;
    } else if (contact_k_ == 0) {
        warm_start_done = false;
    }

    // F. Regresores 
    Vector5d phX, phY, phZ;
    phX << ar*(-Fk_1_[0]), ar*(-Fk_2_[0]), xe[0], xk_1_[0], xk_2_[0];
    phY << ar*(-Fk_1_[1]), ar*(-Fk_2_[1]), xe[1], xk_1_[1], xk_2_[1];
    phZ << ar*(-Fk_1_[2]), ar*(-Fk_2_[2]), xe[2], xk_1_[2], xk_2_[2];

    // G. Predicción ARX (online)
    std::array<double,3> Fp_tej;
    Fp_tej[0] = thX_.dot(phX);
    Fp_tej[1] = thY_.dot(phY);
    Fp_tej[2] = thZ_.dot(phZ);

    // Saturación
    /*Fp_tej[0] = std::max(std::min(Fp_tej[0], tol_sat_), -tol_sat_);
    Fp_tej[1] = std::max(std::min(Fp_tej[1], tol_sat_), -tol_sat_);
    Fp_tej[2] = std::max(Fp_tej[2], -tol_sat_);*/

    // G2. Predicción offline
    Vector5d phX_off, phY_off, phZ_off;
    phX_off << ar*(-Fk_1_off_[0]), ar*(-Fk_2_off_[0]), xe[0], xk_1_[0], xk_2_[0];
    phY_off << ar*(-Fk_1_off_[1]), ar*(-Fk_2_off_[1]), xe[1], xk_1_[1], xk_2_[1];
    phZ_off << ar*(-Fk_1_off_[2]), ar*(-Fk_2_off_[2]), xe[2], xk_1_[2], xk_2_[2];

    std::array<double,3> Fp_tej_off;
    Fp_tej_off[0] = thX_off_.dot(phX_off);
    Fp_tej_off[1] = thY_off_.dot(phY_off);
    Fp_tej_off[2] = thZ_off_.dot(phZ_off);

    // Saturación
    /*Fp_tej_off[0] = std::max(std::min(Fp_tej_off[0], tol_sat_), -tol_sat_);
    Fp_tej_off[1] = std::max(std::min(Fp_tej_off[1], tol_sat_), -tol_sat_);
    Fp_tej_off[2] = std::min(Fp_tej_off[2], -tol_sat_);*/

    //ROS_INFO_THROTTLE(1, "[OFFLINE DEBUG] dz: %4f | Fz_offline: %4f | thZ0: % 4f", dZ_k, Fp_tej_off[2], thZ_off_(0));

    // H. RLS (solo en modo CALIBRATING, en contacto estable) 
    if (mode_ == Mode::CALIBRATING &&
        std::abs(xk_[2]) > 0.0001 &&
        contact_k_ >= RAMP_WIN_)
    {
        double eX = F_tissue(0) - Fp_tej[0];
        double eY = F_tissue(1) - Fp_tej[1];
        double eZ = F_tissue(2) - Fp_tej[2];
        rlsUpdate(thX_, PX_, phX, eX, lambda_rls_);
        rlsUpdate(thY_, PY_, phY, eY, lambda_rls_);
        rlsUpdate(thZ_, PZ_, phZ, eZ, lambda_rls_);
    }

    // Aplicar rampa a la salida
    Fp_tej[0] *= ramp;
    Fp_tej[1] *= ramp;
    //Fp_tej[2] *= ramp;

    Fp_tej_off[0] *= ramp;
    Fp_tej_off[1] *= ramp;
    //Fp_tej_off[2] *= ramp;
    
    ROS_INFO_THROTTLE(1, "[THETAS] OFF_b0: %.4f | ON_b0: %.4f | OFF_a1: %.4f | ON_a1: %.4f ", 
                        thZ_off_(2), thZ_(2), thZ_off_(0), thZ_(0));

    // I. Separación de fuerzas 
    // F_total → frame mundo
    Vector3d F_tot_world = R_Mesa_Auto_ * F_robot;
    Vector3d F_abd_est   = F_tot_world - Vector3d(Fp_tej[0], Fp_tej[1], Fp_tej[2]);
    Vector3d F_abd_off   = F_tot_world - Vector3d(Fp_tej_off[0], Fp_tej_off[1], Fp_tej_off[2]);

    // J. Publicar 
    auto makeMsg = [](double x, double y, double z) {
        std_msgs::Float64MultiArray m;
        m.data = {x, y, z};
        return m;
    };

    // Publicar Online
    pub_tej_est_.publish(makeMsg(Fp_tej[0], Fp_tej[1], Fp_tej[2]));
    pub_abd_est_.publish(makeMsg(F_abd_est(0), F_abd_est(1), F_abd_est(2)));
    // Publicar Offline
    pub_tej_off_.publish(makeMsg(Fp_tej_off[0], Fp_tej_off[1], Fp_tej_off[2]));
    pub_abd_off_.publish(makeMsg(F_abd_off(0), F_abd_off(1), F_abd_off(2)));
    // Señales reales taradas (para comparar en PlotJuggler)
    pub_robot_tared_.publish(makeMsg(F_tot_world(0), F_tot_world(1), F_tot_world(2)));
    pub_tej_real_.publish   (makeMsg(F_tissue(0),    F_tissue(1),    F_tissue(2)));
    pub_abd_real_.publish   (makeMsg(F_abdomen(0),   F_abdomen(1),   F_abdomen(2)));

    // K. Actualizar estados pasados 
    Fk_2_ = Fk_1_;
    Fk_1_ = Fp_tej;   // con rampa, igual que en MATLAB (Fk_1 = Fp_tej(k,:))
    Fk_2_off_ = Fk_1_off_;
    Fk_1_off_ = Fp_tej_off;
    xk_2_ = xk_1_;
    xk_1_ = xe;

}


//  RLS — actualización de un eje
void ForceEstimator::rlsUpdate(Vector5d& th, Matrix5d& P,
                                 const Vector5d& phi,
                                 double error, double lambda)
{
    double denom = lambda + phi.dot(P * phi);
    if (std::abs(denom) < 1e-12) return;
    Vector5d K = (P * phi) / denom;
    th += K * error;
    P   = (P - K * phi.transpose() * P) / lambda;
    P   = (P + P.transpose()) * 0.5;   // simetrizar
}


//  main
int main(int argc, char** argv)
{
    ros::init(argc, argv, "force_estimator");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    ForceEstimator estimator(nh, nh_priv);

    // Multi-hilo para no bloquear callbacks
    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
