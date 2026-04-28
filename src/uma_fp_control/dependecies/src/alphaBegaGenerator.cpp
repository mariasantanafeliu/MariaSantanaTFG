// #include <dependecies/computePoseDes_JM.hpp>
#include "dependecies/alphaBegaGenerator.hpp"

// Eigen::MatrixXd InversaMatriz(Eigen::MatrixXd R);

alphaBegaGenerator::alphaBegaGenerator(){
	ROS_INFO_STREAM("---alphaBegaGenerator---");
  fulcrum_point_goal.x = 0;
  fulcrum_point_goal.y = 0;
  fulcrum_point_goal.z = 0;
  alpha = 0;
  beta = 0;
}	

alphaBegaGenerator::~alphaBegaGenerator(){
    ROS_INFO_STREAM("Leaving gently alphaBegaGenerator...");
}
std::vector<double> alphaBegaGenerator::computeAngles(Eigen::MatrixXd& base_T_dest, Eigen::MatrixXd& base_T_fulcrum){
  fulcrum_point_goal.x = base_T_dest(0,3) - base_T_fulcrum(0,3);
  fulcrum_point_goal.y = base_T_dest(1,3) - base_T_fulcrum(1,3);
  fulcrum_point_goal.z = base_T_dest(2,3) - base_T_fulcrum(2,3);
  alpha = atan2(fulcrum_point_goal.y, fulcrum_point_goal.x);
  beta = atan2(fulcrum_point_goal.z, sqrt(fulcrum_point_goal.x * fulcrum_point_goal.x + fulcrum_point_goal.y * fulcrum_point_goal.y));
  return {alpha, beta};
}