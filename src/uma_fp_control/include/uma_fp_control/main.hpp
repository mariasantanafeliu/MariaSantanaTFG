#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
using Eigen::Vector3d;
#include "cmath"
#include "math.h"
#include <chrono>
#include <std_msgs/Bool.h>
#include "geometry_msgs/Point.h"

#include "force_position_control_dependecies/ftResponse.h"
#include "force_position_control_dependecies/zeroSensorpetition.h"

#include <omni_msgs/OmniFeedback.h>
#include <omni_msgs/OmniState.h>
#include <omni_msgs/OmniButtonEvent.h>

#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include <force_position_control_dependecies/FTSensorRepository.hpp>
#include <force_position_control_dependecies/projects_class.h>
#include <force_position_control_dependecies/omni_teleop.hpp>

#include <utilities/selectTool.hpp>
#include <utilities/computeError.hpp>
#include <utilities/forceControlStrategy.hpp>
#include <utilities/initialize.hpp>
#include <utilities/PIDimplementation.hpp>
#include "utilities/computeT.hpp"

#include "force_position_control/stitches.h"

using namespace std;

// Definición de posiciones
const std::vector<double> punto_vacio = {-135 * DEG_TO_RAD, -90.0 * DEG_TO_RAD, 90.0 * DEG_TO_RAD, -90.0 * DEG_TO_RAD, -90.0 * DEG_TO_RAD, 0.0 * DEG_TO_RAD};
const std::vector<double> punto_vacio2 = {-135 * DEG_TO_RAD, -90.0 * DEG_TO_RAD, 90.0 * DEG_TO_RAD, 0.0 * DEG_TO_RAD, 0.0 * DEG_TO_RAD, 0.0 * DEG_TO_RAD};
const std::vector<double> punto_vacio_tool2 = {-135 * DEG_TO_RAD, -90.0 * DEG_TO_RAD, 90.0 * DEG_TO_RAD, -100.0 * DEG_TO_RAD, 0 * DEG_TO_RAD, 11.0 * DEG_TO_RAD};
const std::vector<double> punto_trocar = {-50.17 * DEG_TO_RAD, -95.25 * DEG_TO_RAD, 94.54 * DEG_TO_RAD,
                                             -90.15 * DEG_TO_RAD, -90 * DEG_TO_RAD, 0 * DEG_TO_RAD}; //180.4
const std::vector<double> punto_trocar2 = {-52.35 * DEG_TO_RAD, -107.16 * DEG_TO_RAD, 102.23 * DEG_TO_RAD,
                                             -83.36 * DEG_TO_RAD, -88.22 * DEG_TO_RAD, 0 * DEG_TO_RAD};
const std::vector<double> punto_trocar3 = {-26 * DEG_TO_RAD, -88.75 * DEG_TO_RAD, 78.32 * DEG_TO_RAD,
                                             -75.57 * DEG_TO_RAD, -102.68 * DEG_TO_RAD, 0 * DEG_TO_RAD};
const std::vector<double> punto_mesa = {-52.4 * DEG_TO_RAD, -73.10 * DEG_TO_RAD, 90.71 * DEG_TO_RAD,
                                             -108.5 * DEG_TO_RAD, -91.98 * DEG_TO_RAD, 0 * DEG_TO_RAD};
const std::vector<double> punto_teleoperacion = {49.73 * DEG_TO_RAD, -84.5 * DEG_TO_RAD, 77 * DEG_TO_RAD,
                                             187.16 * DEG_TO_RAD, -93.3 * DEG_TO_RAD, 1.8 * DEG_TO_RAD};
const std::vector<double> punto_teleoperacion2 = {49.73 * DEG_TO_RAD, -84.5 * DEG_TO_RAD, 90 * DEG_TO_RAD,
                                             265 * DEG_TO_RAD, -90 * DEG_TO_RAD, 1.8 * DEG_TO_RAD};
const std::vector<double> punto_laparoscopio = {-35.33* DEG_TO_RAD, -65.95 * DEG_TO_RAD, 46.49 * DEG_TO_RAD,
                                             -67.02 * DEG_TO_RAD, -108.5 * DEG_TO_RAD, 5.45 * DEG_TO_RAD};
const std::vector<double> punto_curva2 = {-135 * DEG_TO_RAD, -90.0 * DEG_TO_RAD, 90.0 * DEG_TO_RAD, -125.72 * DEG_TO_RAD, -84 * DEG_TO_RAD, 0.0 * DEG_TO_RAD};
// Definición del vector de velocidad x, y, z, dx, dy, dz
std::vector<double> vel = {0., 0., 0., 0., 0., 0.};
std::vector<double> vel2 = {0., 0., 0., 0., 0., 0.};
double velFx =0;
double velFy =0;
Eigen::MatrixXd velLineal(4,1);
Eigen::MatrixXd velAngular(4,1);
Eigen::MatrixXd velLineal2(4,1);
Eigen::MatrixXd velAngular2(4,1);

//fuerzas debajo umbral
bool fuerzasFulcro = false;

// Definición de los TCP que van a utilizarse x, y, z, dx, dy, dz
std::vector<double> tool0 = {0., 0., 0., 0., 0., 0.};
//std::vector<double> TCP1 = {0., 0.0, 0.347, 0., 0., 0.};
std::vector<double> TCP1 = {0., -0.06, 0.324, 0., 0., 0.}; //0.346

std::vector<double> DTTP = {TCP1[0],TCP1[1], TCP1[2]};

// matrices a la muñeca del robot y el fulcro desde el efector final
Eigen::MatrixXd E_T_TTP;
Eigen::MatrixXd E_T_Fp;

/*tf2::Quaternion quaternion;
double roll, pitch, yaw;
double roll_origin, pitch_origin, yaw_origin;*/

//Desplazamiento por haptic
Eigen::MatrixXd deltaHaptica(4,1);
std::vector<double> deltaP = {0,0,0};

//Parametros PID
std::vector<double> diffPose = {0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<double> cumError = {0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<double> lastError = {0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<double> KpVector = {0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<double> KiVector = {0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<double> KdVector = {0.0,0.0,0.0,0.0,0.0,0.0};

std::vector<double> diffPose2 = {0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<double> cumError2 = {0.0,0.0,0.0,0.0,0.0,0.0};
std::vector<double> lastError2 = {0.0,0.0,0.0,0.0,0.0,0.0};

//std::vector<double> Xf = {0.0, 0.0, 0.0, 0.0, 0.0};

//matrices robot 1
Eigen::MatrixXd T_TTP(4,4);
Eigen::MatrixXd T_E(4,4);
Eigen::MatrixXd T_Fp(4,4);
Eigen::MatrixXd T_FpOG(4,4);
Eigen::MatrixXd TTP_dest(4,4);

//matrices robot 2
Eigen::MatrixXd T_TTP2(4,4);
Eigen::MatrixXd T_E2(4,4);
Eigen::MatrixXd T_Fp2(4,4);
Eigen::MatrixXd T_FpOG2(4,4);
Eigen::MatrixXd TTP_dest2;

//posible borrar
/*Eigen::MatrixXd W_DespP(4,1);
Eigen::MatrixXd R_DespP(4,1);
Eigen::MatrixXd T_TTP_origen(4,4);
Eigen::MatrixXd T_TTP_origen_W(4,4);
Eigen::MatrixXd inv_R_T_Fp(4,4);
Eigen::MatrixXd W_DP(4,1);
Eigen::MatrixXd W_DP2(3,1);*/

//INCREMENTO
std::vector<double> DP = {0.0, 0.00, 0.0};
std::vector<double> D = {0., 0., 0.};
/*double DP1=0;
double DP2=0;
double DP3=0;
double Dx2;
double Dy2;
double Dz2;
double Dx;
double Dy;
double Dz;*/

std::vector<double> respuestaSensor = {0, 0, 0, 0, 0, 0};
std::vector<double> respuestaSensor2 = {0, 0, 0, 0, 0, 0};

//feedback haptic
omni_msgs::OmniFeedback envio;
geometry_msgs::Vector3 fuerza;
geometry_msgs::Vector3 posicion;

//fulcrum point
double pAnterior = 0.20;

//previousTime
auto previousTime = std::chrono::system_clock::now();

std::chrono::duration<double> elapsedSeconds;
//Stitches
geometry_msgs::Point positionStitches;
geometry_msgs::Point lastPositionStitches;
std_msgs::Bool arrive;
bool finish = false;
//Stitches
geometry_msgs::Point positionStitches2;
geometry_msgs::Point lastPositionStitches2;

//RPY destino
/*typedef struct {
    double roll;
    double pitch; 
    double yaw;
    } angulo;
angulo destino;
angulo actual;
double RPYdest[3] = {0,0,0};
double RPY[3] = {0,0,0};*/
int Kh=1;