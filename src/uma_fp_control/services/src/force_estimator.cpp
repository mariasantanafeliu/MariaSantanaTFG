#include <services/force_estimator.hpp>
#include <mutex>
#include <cmath>
#include <array>
#include <deque>
#include <algorithm>

namespace {
struct DelayedDefSample {
 std::array<double,3> xk;
 double dZ;
 double ramp;
 int contact_k;
};

std::deque<DelayedDefSample> g_def_buffer;
bool g_warm_start_done = false;

// Estados pasados de entrada para el modelo offline.
// Se mantienen separados para que las correcciones kxy/kz afecten solo al online.
std::array<double,3> g_xk_1_off_input = {0.0, 0.0, 0.0};
std::array<double,3> g_xk_2_off_input = {0.0, 0.0, 0.0};
}

// Coeficientes Butterworth orden 4, fc=3 Hz, fs=125 Hz (Tustin / bilineal)
// Calculados con scipy.signal.butter(4, 3/62.5, 'low') en Python
static const std::vector<double> B_LP = {
 3.37180478e-04, 1.34872191e-03, 2.02308287e-03,
 1.34872191e-03, 3.37180478e-04
};
static const std::vector<double> A_LP = {
 1.0, -3.14515479, 3.72146949,
 -1.96750921, 0.39186971
};


// Constructor
ForceEstimator::ForceEstimator(ros::NodeHandle& nh, ros::NodeHandle& nh_priv)
{
 // Parámetros ROS 
 nh_priv.param<std::string>("config_path", config_path_,
 ros::package::getPath("uma_fp_control") + "/config");
 nh_priv.param<double>("Z_sup", Z_sup_, 0.0605);
 nh_priv.param<double>("Z_adh", Z_adh_, 0.003);
 nh_priv.param<int> ("RAMP_WIN", RAMP_WIN_, 10);
 nh_priv.param<double>("lambda_rls", lambda_rls_, 0.999);

 // Cargar matrices de transformación 
 // T12: auto → darel
 T12_ = loadCSV(config_path_ + "/auto2darel.csv");
 // T23: darel → world
 Matrix4d T23_ = loadCSV(config_path_ + "/darel2world.csv");
 // Z_sup viene de world2tissue (posición Z de la superficie del tejido)
 Matrix4d world2tissue = loadCSV(config_path_ + "/world2tissue.csv");
 Z_sup_ = world2tissue(2,3); // componente Z de traslación

 T13_ = T12_ * T23_;
 T13_(0,3) += 0.0273; // offset hardcodeado igual que en MATLAB
 R_Mesa_Auto_ = T13_.topLeftCorner<3,3>().transpose();
 R_Darel_Mesa_= T23_.topLeftCorner<3,3>();

 //ROS_INFO("[ForceEstimator] Matrices de transformacion cargadas.");
 //ROS_INFO_STREAM("T13 =\n" << T13_);

 // Filtros 
 filt_robot_.init (B_LP, A_LP, 3);
 filt_tissue_.init (B_LP, A_LP, 3);
 filt_abdomen_.init (B_LP, A_LP, 3);
 filt_pos_.init (B_LP, A_LP, 3);

 // Theta inicial 
 initTheta();

 // Publishers 
 pub_tej_est_ = nh.advertise<std_msgs::Float64MultiArray>("tissue_force_estimated", 100);
 pub_abd_est_ = nh.advertise<std_msgs::Float64MultiArray>("abdomen_force_estimated", 100);
 pub_robot_tared_= nh.advertise<std_msgs::Float64MultiArray>("robot_force_tared", 100);
 pub_tej_real_ = nh.advertise<std_msgs::Float64MultiArray>("tissue_force_real", 100);
 pub_abd_real_ = nh.advertise<std_msgs::Float64MultiArray>("abdomen_force_real", 100);
 pub_tej_off_ = nh.advertise<std_msgs::Float64MultiArray>("tissue_force_offline", 100);
 pub_abd_off_ = nh.advertise<std_msgs::Float64MultiArray>("abdomen_force_offline", 100);
 pub_rcm_error_ = nh.advertise<std_msgs::Float64>("rcm_error", 100);
 pub_deformacion_ = nh.advertise<std_msgs::Float64>("deformacion", 100);


 // Servicios 
 srv_tare_ = nh.advertiseService("tare_forces", &ForceEstimator::srvTare, this);
 srv_freeze_ = nh.advertiseService("freeze_theta", &ForceEstimator::srvFreeze, this);

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

 sub_fulcrum_ = nh.subscribe("/auto/fulcrum", 1, 
 &ForceEstimator::fulcrumCb, this);
 sub_trocar_ = nh.subscribe("/darel/pose_topic", 1, 
 &ForceEstimator::trocarCb, this);

 delay_samples_ = static_cast<int>(1.75*125.0);

 //ROS_INFO("[ForceEstimator] Nodo listo. Modo: CALIBRATING");
 //ROS_INFO("[ForceEstimator] Llama a /tare_forces para tarar y luego a /freeze_theta para congelar theta.");
}


// Carga de CSV (4×4)
Matrix4d ForceEstimator::loadCSV(const std::string& path)
{
 Matrix4d M = Matrix4d::Identity();
 std::ifstream f(path);
 if (!f.is_open()) {
 //ROS_ERROR_STREAM("[ForceEstimator] No se puede abrir: " << path);
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


// Theta inicial (convertidos de continuo a discreto, Tustin, igual que MATLAB)
// sys_Z_c = -tf([4050,1384],[1,1.61,0.59])
// sys_X_c = sys_Y_c = -tf([480,590],[1,1.61,1.40])
// Los coeficientes discretos están precalculados aquí para evitar dependencia
// de control_toolbox en ROS.

void ForceEstimator::initTheta()
{
 // Ts = 0.008 s → Tustin: s = 2/Ts * (z-1)/(z+1)
 // Resultados precalculados (idénticos a c2d en MATLAB con 'tustin'):
 //
 // Eje Z: den=[1, -1.9936, 0.9937] num=[-31.6816, 0.0000, 31.6816]
 // Eje X/Y:den=[1, -1.9887, 0.9888] num=[-3.7441, 0.0000, 3.7441]
 //
 // theta = [a1; a2; b0; b1; b2]

 // Z
 thZ_ << -1.98716, 0.98720,
 -16.1182, -0.0440, 16.0742;
 PZ_ = 10.0 * Matrix5d::Identity();

 // X
 thX_ << -1.98711, 0.98720,
 -1.9170, -0.0188, 1.8983;
 PX_ = 10.0 * Matrix5d::Identity();

 // Y (igual que X)
 thY_ = thX_;
 PY_ = PX_;

 //modelo offline
 thX_off_ = thX_;
 thY_off_ = thY_;
 thZ_off_ = thZ_;
}

void ForceEstimator::fulcrumCb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
 if (msg->data.size() < 3) return;
 std::lock_guard<std::mutex> lock(data_mutex_);
 P_fulcro_virtual_ << msg->data[0], msg->data[1], msg->data[2];
 has_fulcrum_ = true;
}

void ForceEstimator::trocarCb(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
 std::lock_guard<std::mutex> lock(data_mutex_);
 if (msg->data.size() >= 16) {
 // Si viene como matriz 4x4 en column-major (igual que la pose), extraemos la traslación
 P_trocar_real_ << msg->data[12], msg->data[13], msg->data[14];
 has_trocar_ = true;
 } else if (msg->data.size() >= 3) {
 // Si viene directamente como un punto cartesiano XYZ
 P_trocar_real_ << msg->data[0], msg->data[1], msg->data[2];
 has_trocar_ = true;
 }
}

// Callbacks
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


// Servicio: Taraje
bool ForceEstimator::srvTare(std_srvs::Trigger::Request&,
 std_srvs::Trigger::Response& res)
{
 // Reset acumuladores
 tare_count_ = 0;
 tare_robot_acc_ = {0,0,0};
 tare_tissue_acc_ = {0,0,0};
 tare_abdomen_acc_ = {0,0,0};
 tare_requested_ = true;
 tared_ = false;

 // Reset filtros para que arranquen limpios tras el taraje
 filt_robot_.reset();
 filt_tissue_.reset();
 filt_abdomen_.reset();
 filt_pos_.reset();

 // Reset estado ARX
 en_cont_ = false;
 contact_k_ = 0;
 fac_cur_ = 0.8;
 xk_ = {0,0,0}; xk_1_ = {0,0,0}; xk_2_ = {0,0,0};
 Fk_1_ = {0,0,0}; Fk_2_ = {0,0,0};
 Fk_1_off_ = {0,0,0}; Fk_2_off_ = {0,0,0};
 g_xk_1_off_input = {0.0, 0.0, 0.0};
 g_xk_2_off_input = {0.0, 0.0, 0.0};
 g_def_buffer.clear();
 g_warm_start_done = false;

 res.success = true;
 res.message = "Taraje iniciado - promediando " + std::to_string(N_TARE_) + " muestras.";
 //ROS_INFO_STREAM("[ForceEstimator] " << res.message);
 return true;
}


// Servicio: Freeze theta (calibración → cirugía)
bool ForceEstimator::srvFreeze(std_srvs::Trigger::Request&,
 std_srvs::Trigger::Response& res)
{
 if (!tared_) {
 res.success = false;
 res.message = "ERROR: haz el taraje primero antes de congelar theta.";
 //ROS_WARN_STREAM("[ForceEstimator] " << res.message);
 return true;
 }
 mode_ = Mode::FROZEN;

 // Al congelar se empieza una nueva fase: limpiamos la cola de deformación
 // y la memoria dinámica para que no arrastre muestras de calibración.
 g_def_buffer.clear();
 g_warm_start_done = false;
 g_xk_1_off_input = {0.0, 0.0, 0.0};
 g_xk_2_off_input = {0.0, 0.0, 0.0};
 en_cont_ = false;
 contact_k_ = 0;
 fac_cur_ = fraw_max_;
 xk_ = {0,0,0}; xk_1_ = {0,0,0}; xk_2_ = {0,0,0};
 Fk_1_ = {0,0,0}; Fk_2_ = {0,0,0};
 Fk_1_off_ = {0,0,0}; Fk_2_off_ = {0,0,0};

 res.success = true;
 res.message = "Theta congelado. Modo SURGERY activado — RLS desactivado.";
 //ROS_INFO_STREAM("[ForceEstimator] " << res.message);
 return true;
}


// Callback pose
void ForceEstimator::poseCb(const std_msgs::Float64MultiArray::ConstPtr& pose_msg)
{
 if (pose_msg->data.size() < 16) return;

 // 1. Extraer posición TCP del world desde T_TTP (column-major, 16 vals) 
 // pose_topic publica T_TTP columna por columna igual que en movimientoTFG.cpp
 /*Matrix4d T_TTP;
 for (int i = 0; i < 4; ++i)
 for (int j = 0; j < 4; ++j)
 T_TTP(j,i) = pose_msg->data[i*4+j];

 std::vector<double> pos_raw = {T_TTP(0,3), T_TTP(1,3), T_TTP(2,3)};*/
 //std::vector<double> pos_actual = {pose_msg->data[12], pose_msg->data[13],pose_msg->data[14]};
 Eigen::Vector3d pos_actual(pose_msg->data[12], pose_msg->data[13], pose_msg->data[14]);

 /*pos_buffer_.push_back(pos_actual);
 Eigen::Vector3d pos_delayed;

 if (pos_buffer_.size() > delay_samples_){
 pos_delayed = pos_buffer_.front();
 pos_buffer_.pop_front();
 } else {
 return;
 }*/

 //std::vector<double> pos_raw = {pos_delayed(0), pos_delayed(1), pos_delayed(2)}; aa
 std::vector<double> pos_raw = {pos_actual(0), pos_actual(1), pos_actual(2)};
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
 //filt_pos_.filter (pos_raw);
 //filt_robot_.filter (F_robot_raw);
 //filt_tissue_.filter (F_tissue_raw);
 //filt_abdomen_.filter(F_abd_raw_v);

 // Inversión de signo en Z (igual que en MATLAB)
 F_tissue_raw[2] = -F_tissue_raw[2];
 F_abd_raw_v [2] = -F_abd_raw_v[2];
 F_robot_raw [2] = F_robot_raw[2];

 // 4. Taraje 
 if (tare_requested_ && !tared_) {
 if (tare_count_ < N_TARE_) {
 for (int i = 0; i < 3; ++i) {
 tare_robot_acc_ [i] += F_robot_raw [i];
 tare_tissue_acc_ [i] += F_tissue_raw [i];
 tare_abdomen_acc_[i] += F_abd_raw_v [i];
 }
 ++tare_count_;
 } else {
 for (int i = 0; i < 3; ++i) {
 offset_robot_ (i) = tare_robot_acc_ [i] / N_TARE_;
 offset_tissue_ (i) = tare_tissue_acc_ [i] / N_TARE_;
 offset_abdomen_(i) = tare_abdomen_acc_[i] / N_TARE_;
 }
 tared_ = true;
 tare_requested_ = false;
 //ROS_INFO("[ForceEstimator] Taraje completado.");
 }
 return; // no procesar hasta completar taraje
 }

 if (!tared_) return; // esperar taraje antes de estimar

 // 5. Aplicar offsets de taraje 
 Vector3d F_robot (F_robot_raw[0] - offset_robot_(0),
 F_robot_raw[1] - offset_robot_(1),
 F_robot_raw[2] - offset_robot_(2));
 Vector3d F_tissue (F_tissue_raw[0] - offset_tissue_(0),
 F_tissue_raw[1] - offset_tissue_(1),
 F_tissue_raw[2] - offset_tissue_(2));
 Vector3d F_abdomen(F_abd_raw_v[0] - offset_abdomen_(0),
 F_abd_raw_v[1] - offset_abdomen_(1),
 F_abd_raw_v[2] - offset_abdomen_(2));

 // Abdomen al frame mundo (igual que en MATLAB: R_Darel_Mesa * Fa)
 Vector3d F_abd_world = R_Darel_Mesa_ * F_abdomen;

 // 6. Procesar muestra (ARX + RLS) 
 Vector3d pos_w(pos_raw[0], pos_raw[1], pos_raw[2]);
 processOneSample(pos_w, F_robot, F_tissue, F_abd_world, V_robot_raw);
}


// Núcleo del algoritmo — equivalente al bucle k de MATLAB
void ForceEstimator::processOneSample(const Vector3d& pos_world,
 const Vector3d& F_robot,
 const Vector3d& F_tissue,
 const Vector3d& F_abdomen,
 const std::vector<double>& V_robot)
{
 (void)V_robot; // En esta versión no se usa la velocidad del topic para no mezclar tiempos.

 // ============================================================
 // A. Deformación geométrica ACTUAL a partir de la pose actual
 // ============================================================
 Eigen::Vector4d pw_h;
 pw_h << pos_world, 1.0;

 Eigen::Vector4d pm_h = T13_.inverse() * pw_h;
 double Pm_x = pm_h(0);
 double Pm_y = pm_h(1);
 double Pm_z = pm_h(2);
 double dZ_now = Pm_z - Z_sup_;

 std::array<double,3> xk_now = {0.0, 0.0, 0.0};

 if (dZ_now <= 0.0) {
 // Contacto con tejido
 if (!en_cont_) {
 pto_X_ = Pm_x;
 pto_Y_ = Pm_y;
 en_cont_ = true;
 prof_mx_ = dZ_now;
 }

 double dXb = Pm_x - pto_X_;
 double dYb = Pm_y - pto_Y_;

 if (dZ_now < prof_mx_) {
 prof_mx_ = dZ_now;
 }

 if (dZ_now > prof_mx_ + 1e-4) {
 // Subiendo: recuperación con adherencia
 double r_rec = Z_adh_ - prof_mx_;
 double rf = std::pow(
 std::max(0.0, std::min(1.0,
 1.0 - (dZ_now - prof_mx_) / r_rec)), 1.3);

 xk_now = {dXb * rf, dYb * rf, dZ_now};
 } else {
 // Bajando o plano
 xk_now = {dXb, dYb, dZ_now};
 }
 }
 else if (dZ_now > 0.0 && dZ_now < Z_adh_ && en_cont_) {
 // Zona de adherencia al salir
 double r_rec = Z_adh_ - prof_mx_;
 double rf = std::pow(
 std::max(0.0, std::min(1.0,
 1.0 - (dZ_now - prof_mx_) / r_rec)), 1.3);

 xk_now = {
 (Pm_x - pto_X_) * rf,
 (Pm_y - pto_Y_) * rf,
 dZ_now * (1.0 - dZ_now / Z_adh_)
 };
 }
 else {
 // Sin contacto geométrico actual.
 // No reseteamos aquí Fk/xk pasados: el reset del modelo se hará
 // cuando el paquete retrasado indique que ya no hay contacto.
 en_cont_ = false;
 pto_X_ = Pm_x;
 pto_Y_ = Pm_y;
 prof_mx_ = 0.0;
 xk_now = {0.0, 0.0, 0.0};
 }

 // ============================================================
 // B. Contador de contacto y rampa ACTUALES
 // ============================================================
 if (dZ_now <= 0.0 && en_cont_) {
 contact_k_ = std::min(contact_k_ + 1, RAMP_WIN_);
 }
 else if (dZ_now > 0.0 && dZ_now < Z_adh_ && en_cont_) {
 contact_k_ = std::max(contact_k_ - 1, 0);
 }
 else {
 contact_k_ = 0;
 }

 double ramp_now = static_cast<double>(contact_k_) / RAMP_WIN_;

 // ============================================================
 // C. Retardo de paquete completo de deformación geométrica
 // Esto sustituye al retardo de la pose.
 // ============================================================
 DelayedDefSample s_now;
 s_now.xk = xk_now;
 s_now.dZ = dZ_now;
 s_now.ramp = ramp_now;
 s_now.contact_k = contact_k_;

 g_def_buffer.push_back(s_now);

 if (g_def_buffer.size() <= static_cast<size_t>(delay_samples_)) {
 return;
 }

 DelayedDefSample s = g_def_buffer.front();
 g_def_buffer.pop_front();

 // A partir de aquí, TODO el modelo trabaja con el paquete retrasado.
 xk_ = s.xk;
 double dZ_model = s.dZ;
 double ramp = s.ramp;
 int contact_k_model = s.contact_k;

 // ============================================================
 // D. Construcción de entradas ONLINE y OFFLINE por separado
 // ============================================================
 double desp_lat = std::sqrt(xk_[0] * xk_[0] + xk_[1] * xk_[1]);

 // Ajustes empíricos que SOLO deben afectar al online.
 // kxy se aplica simétricamente a X e Y.
 const double kxy = 32.0;
 const double kz  = 70.0;

 std::array<double,3> xe_base;
 xe_base[0] = xk_[0] *
 (1.0 + boost_X_gain_ * std::exp(-std::abs(xk_[0]) / boost_X_scale_));
 xe_base[1] = xk_[1] *
 (1.0 + boost_Y_gain_ * std::exp(-std::abs(xk_[1]) / boost_Y_scale_));
 xe_base[2] = xk_[2];

 // Online: usa corrección lateral en X, Y y Z.
 std::array<double,3> xe;
 xe[0] = xe_base[0] * (1.0 + kxy * desp_lat);
 xe[1] = xe_base[1] * (1.0 + kxy * desp_lat);
 xe[2] = xe_base[2] * (1.0 + kz  * desp_lat);

 // Offline: NO usa kxy ni kz. Así el offline queda como modelo fijo de referencia.
 std::array<double,3> xe_off;
 xe_off[0] = xe_base[0];
 xe_off[1] = xe_base[1];
 xe_off[2] = xe_base[2];

 // Mantenemos tu criterio original: rampa en X/Y, no en Z.
 xe[0] *= ramp;
 xe[1] *= ramp;
 // xe[2] *= ramp;

 xe_off[0] *= ramp;
 xe_off[1] *= ramp;
 // xe_off[2] *= ramp;

 // ============================================================
 // E. Reset del modelo cuando el paquete retrasado sale de contacto
 // ============================================================
 if (contact_k_model == 0) {
 Fk_1_ = {0.0, 0.0, 0.0};
 Fk_2_ = {0.0, 0.0, 0.0};
 Fk_1_off_ = {0.0, 0.0, 0.0};
 Fk_2_off_ = {0.0, 0.0, 0.0};
 xk_1_ = {0.0, 0.0, 0.0};
 xk_2_ = {0.0, 0.0, 0.0};
 g_xk_1_off_input = {0.0, 0.0, 0.0};
 g_xk_2_off_input = {0.0, 0.0, 0.0};
 fac_cur_ = fraw_max_;
 g_warm_start_done = false;
 }

 // ============================================================
 // F. Velocidad y trinquete usando deformación ONLINE retrasada
 // ============================================================
 double fs = 1.0 / Ts_;

 double vxe0 = (xe[0] - xk_1_[0]) * fs;
 double vxe1 = (xe[1] - xk_1_[1]) * fs;
 double vxe2 = (xe[2] - xk_1_[2]) * fs;
 (void)vxe2;

 double vel_lat_calc = std::sqrt(vxe0 * vxe0 + vxe1 * vxe1);

 // Evitamos mezclar la deformación retrasada con una velocidad de topic actual.
 double vel_lat = vel_lat_calc;

 double fraw = std::max(fraw_min_, fraw_max_ - vel_lat * fraw_kvel_);

 if (dZ_model < 0.0 && fraw < fac_cur_) {
 fac_cur_ = fraw;
 }
 else if (dZ_model >= 0.0) {
 fac_cur_ = fraw_max_;
 }

 // Si no quieres usar este trinquete, comenta estas dos líneas.
 // Está aplicado simétricamente a X e Y.
 xe[0] *= fac_cur_;
 xe[1] *= fac_cur_;

 // ============================================================
 // G. Warm-start y activación AR con contacto retrasado
 // ============================================================
 double ar = (contact_k_model >= RAMP_WIN_) ? 1.0 : 0.0;

 if (contact_k_model == RAMP_WIN_ && !g_warm_start_done) {
 double E0_XY = -590.0 / 1.40;
 double E0_Z = -1384.0 / 0.59;

 Fk_1_ = {
 E0_XY * xe[0],
 E0_XY * xe[1],
 E0_Z  * xe[2]
 };

 Fk_2_ = Fk_1_;

 // Warm-start offline separado: usa xe_off, sin kxy/kz.
 Fk_1_off_ = {
 E0_XY * xe_off[0],
 E0_XY * xe_off[1],
 E0_Z  * xe_off[2]
 };
 Fk_2_off_ = Fk_1_off_;

 g_warm_start_done = true;
 }
 else if (contact_k_model == 0) {
 g_warm_start_done = false;
 }

 // ============================================================
 // H. Regresores online
 // ============================================================
 Vector5d phX, phY, phZ;

 phX << ar * (-Fk_1_[0]),
 ar * (-Fk_2_[0]),
 xe[0],
 xk_1_[0],
 xk_2_[0];

 phY << ar * (-Fk_1_[1]),
 ar * (-Fk_2_[1]),
 xe[1],
 xk_1_[1],
 xk_2_[1];

 phZ << ar * (-Fk_1_[2]),
 ar * (-Fk_2_[2]),
 xe[2],
 xk_1_[2],
 xk_2_[2];

 // ============================================================
 // I. Predicción ARX online
 // ============================================================
 std::array<double,3> Fp_tej;
 Fp_tej[0] = thX_.dot(phX);
 Fp_tej[1] = thY_.dot(phY);
 Fp_tej[2] = thZ_.dot(phZ);

 // ============================================================
 // J. Regresores y predicción ARX offline
 // Offline usa xe_off y estados de entrada separados para que kxy/kz no le afecten.
 // ============================================================
 Vector5d phX_off, phY_off, phZ_off;

 phX_off << ar * (-Fk_1_off_[0]),
 ar * (-Fk_2_off_[0]),
 xe_off[0],
 g_xk_1_off_input[0],
 g_xk_2_off_input[0];

 phY_off << ar * (-Fk_1_off_[1]),
 ar * (-Fk_2_off_[1]),
 xe_off[1],
 g_xk_1_off_input[1],
 g_xk_2_off_input[1];

 phZ_off << ar * (-Fk_1_off_[2]),
 ar * (-Fk_2_off_[2]),
 xe_off[2],
 g_xk_1_off_input[2],
 g_xk_2_off_input[2];

 std::array<double,3> Fp_tej_off;
 Fp_tej_off[0] = thX_off_.dot(phX_off);
 Fp_tej_off[1] = thY_off_.dot(phY_off);
 Fp_tej_off[2] = thZ_off_.dot(phZ_off);

 // ============================================================
 // K. Publicar deformación retrasada
 // ============================================================
 std_msgs::Float64 def_msg;
 def_msg.data = xk_[2];
 // Para depurar la entrada efectiva online real del modelo, puedes usar:
 // def_msg.data = xe[2];
 pub_deformacion_.publish(def_msg);

 // ============================================================
 // L. RLS usando paquete retrasado y entrada ONLINE
 // ============================================================
 double dx_dt = (xe[0] - xk_1_[0]) * fs;
 double dy_dt = (xe[1] - xk_1_[1]) * fs;
 double dz_dt = (xe[2] - xk_1_[2]) * fs;

 bool push_z = (dz_dt < -1e-4);
 bool mov_x = (std::abs(dx_dt) > 1e-4) && (xk_[0] * dx_dt > 0);
 bool mov_y = (std::abs(dy_dt) > 1e-4) && (xk_[1] * dy_dt > 0);

 double lambda_X = mov_x ? lambda_rls_ : 1.0;
 double lambda_Y = mov_y ? lambda_rls_ : 1.0;
 double lambda_Z = push_z ? lambda_rls_ : 1.0;

 if (mode_ == Mode::CALIBRATING &&
 std::abs(xk_[2]) > 0.0001 &&
 contact_k_model >= RAMP_WIN_)
 {
 double eX = F_tissue(0) - Fp_tej[0];
 double eY = F_tissue(1) - Fp_tej[1];
 double eZ = F_tissue(2) - Fp_tej[2];

 if (mov_x) rlsUpdate(thX_, PX_, phX, eX, lambda_X);
 if (mov_y) rlsUpdate(thY_, PY_, phY, eY, lambda_Y);
 if (push_z) rlsUpdate(thZ_, PZ_, phZ, eZ, lambda_Z);
 }

 // ============================================================
 // M. Rampa de salida
 // ============================================================
 Fp_tej[0] *= ramp;
 Fp_tej[1] *= ramp;
 // Fp_tej[2] *= ramp;

 Fp_tej_off[0] *= ramp;
 Fp_tej_off[1] *= ramp;
 // Fp_tej_off[2] *= ramp;

 // ============================================================
 // N. Separación de fuerzas
 // ============================================================
 Vector3d F_tot_world = R_Mesa_Auto_ * F_robot;

 Vector3d F_abd_est = F_tot_world - Vector3d(
 Fp_tej[0],
 Fp_tej[1],
 Fp_tej[2]
 );

 Vector3d F_abd_off = F_tot_world - Vector3d(
 Fp_tej_off[0],
 Fp_tej_off[1],
 Fp_tej_off[2]
 );

 // ============================================================
 // O. Error de fulcro RCM
 // ============================================================
 Eigen::Vector3d p_virt, p_real;
 bool rcm_ready = false;

 {
 std::lock_guard<std::mutex> lock(data_mutex_);
 p_virt = P_fulcro_virtual_;
 p_real = P_trocar_real_;
 rcm_ready = has_fulcrum_ && has_trocar_;
 }

 if (rcm_ready) {
 Eigen::Vector4d p_virt_auto_h;
 p_virt_auto_h << p_virt, 1.0;
 Eigen::Vector4d p_virt_mundo = T13_.inverse() * p_virt_auto_h;

 Eigen::Vector4d p_real_darel_h;
 p_real_darel_h << p_real, 1.0;
 Eigen::Vector4d p_real_auto = T12_ * p_real_darel_h;
 Eigen::Vector4d p_real_mundo = T13_.inverse() * p_real_auto;

 p_real_mundo(2) += 0.024; // descalibración z darel

 double error_rcm = (p_real_mundo(2) - p_virt_mundo(2)) * 1000.0;

 std_msgs::Float64 err_msg;
 err_msg.data = error_rcm;
 pub_rcm_error_.publish(err_msg);
 }

 // ============================================================
 // P. Publicar
 // ============================================================
 auto makeMsg = [](double x, double y, double z) {
 std_msgs::Float64MultiArray m;
 m.data = {x, y, z};
 return m;
 };

 pub_tej_est_.publish(makeMsg(Fp_tej[0], Fp_tej[1], Fp_tej[2]));
 pub_abd_est_.publish(makeMsg(F_abd_est(0), F_abd_est(1), F_abd_est(2)));

 pub_tej_off_.publish(makeMsg(Fp_tej_off[0], Fp_tej_off[1], Fp_tej_off[2]));
 pub_abd_off_.publish(makeMsg(F_abd_off(0), F_abd_off(1), F_abd_off(2)));

 pub_robot_tared_.publish(makeMsg(F_tot_world(0), F_tot_world(1), F_tot_world(2)));
 pub_tej_real_.publish(makeMsg(F_tissue(0), F_tissue(1), F_tissue(2)));
 pub_abd_real_.publish(makeMsg(F_abdomen(0), F_abdomen(1), F_abdomen(2)));

 // ============================================================
 // Q. Actualizar estados pasados del modelo
 // ============================================================
 Fk_2_ = Fk_1_;
 Fk_1_ = Fp_tej;

 Fk_2_off_ = Fk_1_off_;
 Fk_1_off_ = Fp_tej_off;

 xk_2_ = xk_1_;
 xk_1_ = xe;

 g_xk_2_off_input = g_xk_1_off_input;
 g_xk_1_off_input = xe_off;
}


// RLS — actualización de un eje
void ForceEstimator::rlsUpdate(Vector5d& th, Matrix5d& P,
 const Vector5d& phi,
 double error, double lambda)
{
 double denom = lambda + phi.dot(P * phi);
 if (std::abs(denom) < 1e-12) return;
 Vector5d K = (P * phi) / denom;
 th += K * error;
 P = (P - K * phi.transpose() * P) / lambda;
 P = (P + P.transpose()) * 0.5; // simetrizar
}


// main
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