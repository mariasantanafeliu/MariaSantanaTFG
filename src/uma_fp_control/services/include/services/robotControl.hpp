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
#include <omni_msgs/OmniFeedback.h>
#include <omni_msgs/OmniState.h>
#include <omni_msgs/OmniButtonEvent.h>
#include <iostream>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include "uma_fp_control/ftResponse.h"
#include "uma_fp_control/zeroSensorpetition.h"
#include <dependecies/computeError.hpp>
#include <dependecies/computePID.hpp>
#include <dependecies/computePoseDes_JM.hpp>
#include <dependecies/computeT.hpp>
#include <dependecies/forceControlStrategy.hpp>
#include <dependecies/initialize.hpp>
#include <dependecies/selectTool.hpp>
#include <dependecies/ur_script.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Pose.h"
#include "dependecies/hex_ft_udp.hpp"
#define DEG_TO_RAD 0.017453293
#define RAD_TO_DEG 57.295779513

class ur3Control
{       
	private:
                ur_script* ur;
                //La lista de inicialización en el constructor de una clase en C++ se utiliza para inicializar los miembros de la clase.
                 //No se utiliza para declarar variables, ya que las variables miembro deben haber sido declaradas previamente en la 
                 //definición de la clase. Para poder usarlo en la lista de inicialización, debe haber sido declarado en la clase
                //ur_script ur(prefix);
                UMA_trans* tr;
                //TF_read* tf;
                //projects_class moveit;
                PoseDesJM* Pdest; //A LO MEJOR SE QUITA
                //HEX_Sensor* ft;
                Init* init;
                ErrorPose* error;
                forceControl* Kforce;
                FTSensor* ftSensor;
                //PIDimplementation* pidVel;
                //PIDController controlPosition;
                Cinematic* T;
                selectTool* tool;
                std::string prefix_in;
                PIDController* controlPosition;
                tf::TransformListener* listener; // Cambiar a puntero
        public:
		ur3Control(ur_script * urScript, UMA_trans *umaTf, PoseDesJM *estimatePose,
                        Init *initRobot, ErrorPose *composeError, forceControl *composeforceControl, 
                        PIDController *composePID, Cinematic *robotTransf, selectTool *selectTool, tf::TransformListener* tf_listener, std::string t_prefix);
                /*ur3Control(ur_script * urScript, UMA_trans *umaTf, TF_read *readtfRobot, PoseDesJM *estimatePose,
                        Init *initRobot, ErrorPose *composeError, forceControl *composeforceControl, 
                        PIDController *composePID, Cinematic *robotTransf, selectTool *selectTool, std::string t_prefix);*/
		~ur3Control();
                //variables
                        //init
                std::vector<double> tool0 = {0., 0., 0., 0., 0., 0.};
                std::vector<double> puntoDestinoRobot;
                        //position control
                geometry_msgs::Point desPose;
                geometry_msgs::Point last_desPose;
                        //force control
                bool fulcrumForces = false;
                double p_estimado;
		std::vector<double> HEXForce = {0,0,0};
		std::vector<double> HEXtorque = {0,0,0};
                        //matrix
                Eigen::MatrixXd E_T_TTP;
                Eigen::MatrixXd E_T_Fp;
                Eigen::MatrixXd T_E;
                Eigen::MatrixXd T_Fp;
                Eigen::MatrixXd T_TTP;
                Eigen::MatrixXd TTP_dest;
                        //error
                std::vector<double> diffPose = {0.0,0.0,0.0,0.0,0.0,0.0};
                        //pid vel
                std::vector<double> velVector = {0.0,0.0,0.0,0.0,0.0,0.0};
                std::vector<double> deltaXF = {0.0,0.0,0.0};
                Eigen::MatrixXd velLineal{Eigen::MatrixXd::Zero(4, 1)};
                Eigen::MatrixXd velAngular{Eigen::MatrixXd::Zero(4, 1)};
                //last
                bool fuerzasFulcro = false;
                Eigen::MatrixXd T_FpOG;
                std::vector<double> TCP;
                //Haptic
                Eigen::MatrixXd transformRobotbaseToHaptic;
                Eigen::MatrixXd hapticDisplacement;
                Eigen::MatrixXd hapticRotation;
                Eigen::MatrixXd robotbaseDisplacement;
                Eigen::MatrixXd robotbaseRotation;
                std::vector<double> deltaP = {0,0,0};
                //ROS
                ros::NodeHandle nh_;
                ros::Subscriber goal_pos_sub_;
                ros::Subscriber initial_pose_sub_;
                ros::Subscriber force_sensor_sub_;
                ros::Subscriber surface_sub_;
                ros::Publisher ttp_pub_;
                ros::Publisher error_pub_;
                std_msgs::Float64MultiArray pose_;
                std_msgs::Float64MultiArray error_;
                //TF
                //tf::TransformListener listener;
                tf::StampedTransform tf_pose;
                geometry_msgs::Pose pose_msg;
		tf::Quaternion quaternion;
                double roll, pitch, yaw;
                double X, Y, Z;
		double qX, qY, qZ, qW;
                std::string base_name;
                std::string tool_name;
                std::string efector_name;
                Eigen::MatrixXd readTransform(std::string base,std::string tool0);
                //callback
                void cb_stitchCallback(const geometry_msgs::Point::ConstPtr& msg);
                void cb_readForceTorque(const uma_fp_control::ftResponse& msg);
                void cb_readSurface(const geometry_msgs::Point::ConstPtr& msg);
                //function
                void initializeRobot(int type, double p_estimado, std::vector<double> initPosition);
                void computeRobotCinematic(bool flagFp);
                void computeForceControl(double Kf);
                void computePoseError();
                void computeRobotVel();
};