#include <dependecies/jacobianss.hpp>

urJacobian::urJacobian() {
    // Initialize the robot model
    //robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    //kinematic_model = robot_model_loader.getModel();
    
    // Get the joint model group
    //joint_model_group = kinematic_model->getJointModelGroup("manipulator");
    
    // Set the end effector
    //move_group.setEndEffectorLink("tool0");
    
    // Initialize reference point
    //The end-effector is a physical object, not just a point. The reference_point_position sets where on it the velocities are calculated. Currently set to (0,0,0), it computes at the origin, but you may adjust it for tool control, a key point on a gripper, or force control at a contact point.
    //reference_point_position = Eigen::Vector3d(0.0, 0.0, 0.0);
}
urJacobian::~urJacobian() {
    // Destructor implementation (can be empty if no manual cleanup is needed)
}
//funciones
/*Eigen::MatrixXd urJacobian::URjacobian(){
    robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
    kinematic_state->getJacobian(joint_model_group, // Which joints to consider
                            kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()), // The end-effector link
                            reference_point_position, // Where on the end-effector
                            jacobian); // Output matrix
    return jacobian;
}*/
Eigen::Matrix<double, 6, 6> urJacobian::URjacobian_crafted(const Eigen::Matrix<double, 6, 3>& DH) {
    Eigen::Matrix<double, 6, 6> J = Eigen::Matrix<double, 6, 6>::Zero();
    std::vector<double> theta = {0,0,0,0,0,0};//move_group.getCurrentJointValues();
    
    J(0,0) = DH(3,2) * cos(theta[0]) + sin(theta[0]) * (DH(1,0) * sin(theta[1]) + DH(2,0) * sin(theta[1] + theta[2]) + DH(4,2) * sin(theta[1] + theta[2] + theta[3]));
    J(0,1) = -cos(theta[0]) * (DH(1,0) * cos(theta[1]) + DH(2,0) * cos(theta[1] + theta[2]) + DH(4,2) * cos(theta[1] + theta[2] + theta[3]));
    J(0,2) = -cos(theta[0]) * (DH(2,0) * cos(theta[1] + theta[2]) + DH(4,2) * cos(theta[1] + theta[2] + theta[3]));
    J(0,3) = -DH(4,2) * cos(theta[0]) * cos(theta[1] + theta[2] + theta[3]);
    
    J(1,0) = DH(3,2) * sin(theta[0]) - cos(theta[0]) * (DH(1,0) * sin(theta[1]) + DH(2,0) * sin(theta[1] + theta[2]) + DH(4,2) * sin(theta[1] + theta[2] + theta[3]));
    J(1,1) = -sin(theta[0]) * (DH(1,0) * cos(theta[1]) + DH(2,0) * cos(theta[1] + theta[2]) + DH(4,2) * cos(theta[1] + theta[2] + theta[3]));
    J(1,2) = -sin(theta[0]) * (DH(2,0) * cos(theta[1] + theta[2]) + DH(4,2) * cos(theta[1] + theta[2] + theta[3]));
    J(1,3) = -DH(4,2) * sin(theta[0]) * cos(theta[1] + theta[2] + theta[3]);
    
    J(2,1) = -DH(1,0) * sin(theta[1]) - DH(2,0) * sin(theta[1] + theta[2]) - DH(4,2) * sin(theta[1] + theta[2] + theta[3]);
    J(2,2) = -DH(2,0) * sin(theta[1] + theta[2]) - DH(4,2) * sin(theta[1] + theta[2] + theta[3]);
    J(2,3) = -DH(4,2) * sin(theta[1] + theta[2] + theta[3]);
    
    J(3,1) = sin(theta[0]);
    J(3,2) = sin(theta[0]);
    J(3,3) = sin(theta[0]);
    J(3,4) = -cos(theta[0]) * sin(theta[1] + theta[2] + theta[3]);
    J(3,5) = sin(theta[0]) * cos(theta[4]) + cos(theta[0]) * cos(theta[1] + theta[2] + theta[3]) * sin(theta[4]);
    
    J(4,1) = -cos(theta[0]);
    J(4,2) = -cos(theta[0]);
    J(4,3) = -cos(theta[0]);
    J(4,4) = -sin(theta[0]) * sin(theta[1] + theta[2] + theta[3]);
    J(4,5) = -cos(theta[0]) * cos(theta[4]) + sin(theta[0]) * cos(theta[1] + theta[2] + theta[3]) * sin(theta[4]);
    
    J(5,0) = 1;
    J(5,4) = cos(theta[1] + theta[2] + theta[3]);
    J(5,5) = sin(theta[1] + theta[2] + theta[3]) * sin(theta[4]);
    
    return J;
}
Eigen::MatrixXd urJacobian::ahJacobian(double a, double b, double p, double L){
    // Precalcular valores trigonométricos
    double ca = cos(a);
    double cb = cos(b);
    double sa = sin(a);
    double sb = sin(b);
    double pL = p - L;

    // Crear JP_AH (3x3 matriz)
    Eigen::MatrixXd JP_AH(3, 3);
    JP_AH(0, 0) = -pL * sa * cb;
    JP_AH(0, 1) = -pL * ca * sb;
    JP_AH(0, 2) = ca * cb;
    JP_AH(1, 0) = pL * cb * ca;
    JP_AH(1, 1) = -pL * sa * sb;
    JP_AH(1, 2) = sa * cb;
    JP_AH(2, 0) = 0;
    JP_AH(2, 1) = pL * cb;
    JP_AH(2, 2) = sb;

    // Crear JO_AH (3x3 matriz)
    Eigen::MatrixXd JO_AH(3, 3);
    JO_AH << 0, 0, 0,
            0, 1, 0,
            1, 0, 0;

    // Concatenar JP_AH y JO_AH verticalmente para formar J (6x3 matriz)
    Eigen::MatrixXd J(6, 3);
    J << JP_AH,
        JO_AH;

    return J;
}
