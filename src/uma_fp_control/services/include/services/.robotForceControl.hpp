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
#include <dependecies/fulcrumEstimation.hpp>
#include "dependecies/alphaBegaGenerator.hpp"
#include <dependecies/fuzzySystem.hpp>
#include <tf/transform_listener.h>
#include "geometry_msgs/Pose.h"
#define DEG_TO_RAD 0.017453293
#define RAD_TO_DEG 57.295779513

class ur3ForceControl
{       
	private:
                ur_script* ur;
                //La lista de inicialización en el constructor de una clase en C++ se utiliza para inicializar los miembros de la clase.
                 //No se utiliza para declarar variables, ya que las variables miembro deben haber sido declaradas previamente en la 
                 //definición de la clase. Para poder usarlo en la lista de inicialización, debe haber sido declarado en la clase
                UMA_trans* tr;
                PoseDesJM* Pdest;
                Init* init;
                ErrorPose* error;
                forceControl* Kforce;
                Cinematic* T;
                selectTool* tool;
                std::string prefix_in;
                PIDController* controlPosition;
                PIDController* controlForce;
                fulcrum* fulcrumEstimation;
                alphaBegaGenerator* alphaBetaGenerator;
                //FuzzySystem* fuzzySystem; 
        public:
		ur3ForceControl(ur_script * urScript, UMA_trans *umaTf, PoseDesJM *estimatePose,
                        Init *initRobot, ErrorPose *composeError, forceControl *composeforceControl,
                        PIDController *composePID, PIDController *composePID2, Cinematic *robotTransf, 
                        selectTool *selectTool, fulcrum *fp, std::string t_prefix);
                /*ur3ForceControl(ur_script * urScript, UMA_trans *umaTf, TF_read *readtfRobot, PoseDesJM *estimatePose,
                        Init *initRobot, ErrorPose *composeError, forceControl *composeforceControl, 
                        PIDController *composePID, Cinematic *robotTransf, selectTool *selectTool, std::string t_prefix);*/
		~ur3ForceControl();
                //variables
                        //init
                std::vector<double> tool0 = {0., 0., 0., 0., 0., 0.};
                std::vector<double> puntoDestinoRobot;
                        //position control
                geometry_msgs::Point desPose;
                geometry_msgs::Point desPoseCom;
                geometry_msgs::Point last_desPose;
                        //force control
                bool fulcrumForces = false;
                bool first_force_reading = true;
                std::vector<double> bias_force = {0,0,0};
		std::vector<double> bias_torque = {0,0,0};
                double p_estimado;
		std::vector<double> HEXForce = {0,0,0};
		std::vector<double> HEXtorque = {0,0,0};
                Eigen::Vector3d forceRobotTTP;
                Eigen::Vector3d torqueRobotTTP;
                Eigen::Vector3d forceRobotBase;
                Eigen::Vector3d torqueRobotBase;
                bool move = false;
                bool first_time = true;
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
                //ROS
                ros::NodeHandle nh_;
                ros::Subscriber goal_pos_sub_;
                ros::Subscriber initial_pose_sub_;
                ros::Subscriber force_sensor_sub_;
                ros::Subscriber surface_sub_;
                ros::Publisher ttp_pub_;
                ros::Publisher vel_pub_;
                ros::Publisher te_pub_;
                ros::Publisher error_pub_;
                ros::Publisher reset_sensor_pub_; //SOLO SI LEES LA FUERZA DE UR3
                ros::Publisher force_pub_;
                ros::Publisher effector_force_pub_;
                std_msgs::Float64MultiArray pose_, poseE_, array_vel, force_msg_, force_effector_msg_;
                std_msgs::Float64MultiArray error_;
                //TF
                tf::TransformListener listener;
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
                //void cb_readForceTorque(const uma_fp_control::ftResponse& msg);
                void cb_readForceTorque(const geometry_msgs::Twist::ConstPtr& msg);
                void cb_readSurface(const geometry_msgs::Point::ConstPtr& msg);
                //function
                void initializeRobot(int type, double p_estimado, double tool_length, std::vector<double> initPosition);
                void computeRobotCinematic(FuzzySystem* fuzzySystem, bool flagFp,int type);
                void computeForceControl(double Kf);
                void computePoseError();
                void computeRobotVel();
                //fulcro
                void computeFulcrum(FuzzySystem* fuzzySystem, double p_anterior);
};