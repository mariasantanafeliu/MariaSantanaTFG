#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h> 

#pragma once
class alphaBegaGenerator
{
	public:
		alphaBegaGenerator();//(std::vector<double> P_Fp, Eigen::MatrixXd T_TTP_origen, std::vector<double> DP, std::vector<double> RPY); //ros::NodeHandle n
		~alphaBegaGenerator();
	
		//funciones
		std::vector<double> computeAngles(Eigen::MatrixXd& base_T_dest, Eigen::MatrixXd& base_T_fulcrum);
		
		//Variables
		double alpha, beta;
		geometry_msgs::Point fulcrum_point_goal;	
};
