#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/Dense>
#include <cmath>
using Eigen::Vector3d;

class urJacobian{       
	private:
        //moveit::planning_interface::MoveGroupInterface move_group;
        //robot_model::RobotModelPtr kinematic_model;
        //const robot_model::JointModelGroup* joint_model_group;
        //Eigen::Vector3d reference_point_position;
        Eigen::MatrixXd jacobian;
    public:
		urJacobian();//const std::string& planning_group = "manipulator");
		~urJacobian();
        //variables
        Eigen::MatrixXd Jacobian;
		//Eigen::MatrixXd URjacobian();
		Eigen::Matrix<double, 6, 6> URjacobian_crafted(const Eigen::Matrix<double, 6, 3>& DH);
		Eigen::MatrixXd ahJacobian(double a, double b, double p, double L);	
};
