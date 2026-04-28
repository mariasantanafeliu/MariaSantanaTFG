#include <services/jacobianControl.hpp>

jacobianControl::jacobianControl(ur_script* urScript, UMA_trans* umaTf,
                       Init* initRobot, PIDController* composePID, selectTool* selectTool, fulcrum* fp, FTSensor* ftSensor,  tf::TransformListener* tf_listener, std::string t_prefix) : 
                       ur(urScript), tr(umaTf), init(initRobot), controlPosition(composePID), tool(selectTool), fulcrumEstimation(fp), ftSensor(ftSensor), listener(tf_listener){//lista de inicialización
    std::cout << t_prefix << " control build" << std::endl;
    prefix_in = t_prefix;
    // Subscripciones a los topics
    goal_pos_sub_ = nh_.subscribe("/"+t_prefix+"/coordinator/goal_position", 1000, &jacobianControl::cb_stitchCallback, this);
    // Publicacion topics
    ttp_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("pose_topic", 1000);
    te_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("effectorFinal_topic", 1000);
    vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("velocity_topic", 1000);
    currentPolar_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("polar/current", 1000);
    desPolar_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("polar/des", 1000);
    deltaPolar_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("polar/delta", 1000);
    fulcrum_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("fulcrum", 1000);
    desPose.x = 0.;
    desPose.y = 0.;
    desPose.z = 0.;
    delta_polar << 0,0,0;
    delta_cartesian = Eigen::MatrixXd::Zero(6, 1); // Matriz dinámica 6x1 inicializada con ceros
    d = Eigen::MatrixXd::Zero(6, 1); // Matriz dinámica 6x1 inicializada con ceros
}
jacobianControl::~jacobianControl(){
    std::cout <<"Leaving gently jacobianControl..."<< std::endl;
}
//#######################callback##########################
void jacobianControl::cb_stitchCallback(const geometry_msgs::Point::ConstPtr& msg){
    desPose.x = msg->x;
    desPose.y = msg->y;
    desPose.z = msg->z;
    desPoseReceived = true;
    show = true;
    std::cout << "T_TTP: " << T_TTP << std::endl;
    std::cout << "desPose: " << desPose << std::endl;
    std::cout << "fulcrum_position: " << fulcrum_position << std::endl;
    std::cout << "------------------------------------------------------------------------------------------------------------------------------------------------------------" <<  std::endl;
}
//#######################function##########################
Eigen::MatrixXd jacobianControl::readTransform(std::string base,std::string tool0){
    Eigen::MatrixXd T(4,4);
    try {
        listener->waitForTransform(base, tool0, ros::Time(0), ros::Duration(1.0));
        listener->lookupTransform(base, tool0, ros::Time(0), tf_pose);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR_STREAM("TF Exception: " << ex.what());
        return T;  // ❗ Salimos: no seguimos con datos corruptos
    }
    qX = tf_pose.getRotation().x();
    qY = tf_pose.getRotation().y();
    qZ = tf_pose.getRotation().z();
    qW = tf_pose.getRotation().w();
    X = tf_pose.getOrigin().x();
    Y = tf_pose.getOrigin().y();
    Z = tf_pose.getOrigin().z();
    T << (1-2*(qY*qY+qZ*qZ)), (2*(qX*qY-qW*qZ)), 2*(qX*qZ+qW*qY), X,
         (2*(qX*qY+qW*qZ)), (1-2*(qX*qX+qZ*qZ)), 2*(qY*qZ-qW*qX), Y,
         2*(qX*qZ-qW*qY), 2*(qY*qZ+qW*qX), (1-2*(qX*qX+qY*qY)), Z,
         0,0,0,1;
    return T;
}
// funcion that retunr a geometric position
geometry_msgs::Point jacobianControl::computeFulcrum(Eigen::MatrixXd E_T_Fp, Eigen::MatrixXd T_E){
    geometry_msgs::Point fulcrum_position;
    Eigen::MatrixXd T(4,4);
    T = T_E * E_T_Fp;
    fulcrum_position.x = T(0,3);
    fulcrum_position.y = T(1,3);
    fulcrum_position.z = T(2,3);
    return fulcrum_position;
}
//round delta_cartesian
void jacobianControl::roundDeltaCartesian(Eigen::VectorXd& delta_cartesian) { 
    if (delta_cartesian.size() != 6) {
        std::cerr << "Error: El vector debe tener exactamente 6 elementos." << std::endl;
        return;
    }
    for (int i = 0; i < delta_cartesian.size(); ++i) {
        double rounded = std::round(delta_cartesian[i] * 10000.0) / 10000.0;
        delta_cartesian[i] = (std::abs(rounded) < 1e-5) ? 0.0 : rounded;
    }
}
Eigen::MatrixXd jacobianControl::computeInverseJacobian(double a, double b, double p, double L){
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

    //inverse de JP_AH
    Eigen::MatrixXd JP_AH_inv = JP_AH.inverse(); //completeOrthogonalDecomposition().pseudoInverse();

    return JP_AH_inv;
}
Eigen::MatrixXd jacobianControl::computeJacobian(double a, double b, double p, double L){
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
    //J = JO_AH;
    return J;
}
void jacobianControl::moveToDesireJacobOrientation_init(Eigen::MatrixXd T_TTP,  std::vector<double> polarCoordinates, double tool_length){
    std::cout << "---------------------------------------" << std::endl;
    std::cout << "estamos en moveToDesireJacobOrientation_init" << std::endl;
    std::vector<double> eulerZYX(3), error(3);
    Eigen::Vector3d vel_orientation_tool, vel_orientation_base;
    std::vector<double> vel = {0,0,0,0,0,0};
    Eigen::MatrixXd dR(4,4);
    Eigen::MatrixXd T_dest;
    /*eulerZYX[0]=std::atan2(T_TTP(1,0),T_TTP(0,0)); //Z
    eulerZYX[1]=std::atan2(-T_TTP(2,0),sqrt(pow(T_TTP(0,0),2)+pow(T_TTP(1,0),2))); //Y
    eulerZYX[2]=std::atan2(T_TTP(2,1),T_TTP(2,2));*/ //X
    std::cout << "T_TTP: " << T_TTP << std::endl;
    std::cout << T_TTP(0,3) << ", " << T_TTP(1,3) << ", " << T_TTP(2,3) << ',' << std::atan2(T_TTP(2,1),T_TTP(2,2)) << ',' << std::atan2(-T_TTP(2,0),sqrt(pow(T_TTP(0,0),2)+pow(T_TTP(1,0),2))) << ',' << std::atan2(T_TTP(1,0),T_TTP(0,0)) << std::endl;
    T_dest = tr->desp({T_TTP(0,3), T_TTP(1,3), T_TTP(2,3)}) * tr->rotZ(polarCoordinates[0]) * tr->rotY(-3.141592653589793/2-polarCoordinates[1]);
    dR = (T_dest.topLeftCorner<3,3>()).transpose() * T_TTP.topLeftCorner<3,3>();
    error[0] = atan2(dR(2,1), dR(2,2));
    error[0]*=1000;
    error[0]=round(error[0]);
    error[0]/=1000;
    error[1] = std::atan2(-dR(2,0),sqrt(pow(dR(0,0),2)+pow(dR(1,0),2)));
    error[1]*=1000;
    error[1]=round(error[1]);
    error[1]/=1000;
    error[2] = std::atan2(dR(1,0),dR(0,0));
    error[2]*=1000;
    error[2]=round(error[2]);
    error[2]/=1000;
    if (error[2]<0){
        error[2] = error[2] + 3.141592653589793;
    }
    std::cout << "error: " << error[0] << " " << error[1] << " " << error[2] << std::endl;
    vel_orientation_tool << -error[0], -error[1], error[2];
    vel_orientation_base = T_TTP.topLeftCorner<3,3>() * vel_orientation_tool;
    //std::cout << "vel_orientation_tool: " << vel_orientation_tool << std::endl;
    //std::cout << "vel_orientation_base: " << vel_orientation_base << std::endl;
    vel = {0,0,0, vel_orientation_base(0),vel_orientation_base(1),vel_orientation_base(2)};
    //ur->speedl(vel, 0.1, 0.1);

    double epsilon = 0.05;
    if (std::abs(error[2]) < epsilon || std::abs(error[2]) > 3.12) {
        std::cout << "\033[1;33mARMADILLO\033[0m" << std::endl; // Texto amarillo
        inOrientation = true;
    } else {
        ur->speedl(vel, 0.1, 0.1);
    }
}

void jacobianControl::moveToDesirePosition_jacobian(Eigen::MatrixXd T_TTP, geometry_msgs::Point nextPose, std::vector<double> des_polarCoordinates, std::vector<double> polarCoordinates, double L){
    std::cout << "---------------------------------------" << std::endl;
    //get dynamic linear velocity
    delta_position << (nextPose.x - T_TTP(0,3)),
                    (nextPose.y - T_TTP(1,3)),
                    (nextPose.z - T_TTP(2,3));
    delta_position[0]  = std::round(delta_position[0] * 10000.0) / 10000.0;
    delta_position[1]  = std::round(delta_position[1] * 10000.0) / 10000.0;
    delta_position[2]  = std::round(delta_position[2] * 10000.0) / 10000.0;
    /*inverseJacobian = computeInverseJacobian(current_polar_angles[0], current_polar_angles[1], current_polar_angles[2], L);
    delta_polar = inverseJacobian * delta_position;*/
    std::vector<double> current_angle(3), des_angle(3), error(3);
    Eigen::Vector3d vel_orientation_tool, vel_orientation_base;
    Eigen::MatrixXd R_dest(4,4), dR(4,4), R_current(4,4);
    //std::cout << "Pose: " << T_TTP(0,3) << ", " << T_TTP(1,3) << ", " << T_TTP(2,3) <<'<'<<current_polar_angles[0] << ',' << current_polar_angles[1] << ',' << current_polar_angles[2] << std::endl;
    //
    delta_polar << (des_polarCoordinates[0] - current_polar_angles[0]),
                    (des_polarCoordinates[1] - current_polar_angles[1]),
                    (des_polarCoordinates[2] - current_polar_angles[2]);
    /*Jacobian = computeJacobian(current_polar_angles[0], current_polar_angles[1], current_polar_angles[2], L);
    delta_cartesian = Jacobian * delta_polar;
    roundDeltaCartesian(delta_cartesian);*/
    //------test------
    //R_dest = tr->rotZ(current_polar_angles[0] + delta_polar[0]) * tr->rotY(-3.141592653589793/2-current_polar_angles[1] - delta_polar[1]);
    R_current = tr->desp({T_TTP(0,3), T_TTP(1,3), T_TTP(2,3)}) * tr->rotZ(current_polar_angles[0]) * tr->rotY(-3.141592653589793/2-current_polar_angles[1]);
    R_dest = tr->desp({nextPose.x,nextPose.y,nextPose.z}) * tr->rotZ(des_polarCoordinates[0]) * tr->rotY(-3.141592653589793/2-des_polarCoordinates[1]);
    //compose Error -> Igual que en "computeError.cpp"
    dR = (T_TTP.topLeftCorner<3,3>()).transpose() * (R_dest.topLeftCorner<3,3>());
    error[0] = std::atan2(dR(2,1), dR(2,2));
    error[0]*=1000;
    error[0]=round(error[0]);
    error[0]/=1000;
    error[1] = std::atan2(-dR(2,0),sqrt(pow(dR(0,0),2)+pow(dR(1,0),2)));
    error[1]*=1000;
    error[1]=round(error[1]);
    error[1]/=1000;
    error[2] = std::atan2(dR(1,0),dR(0,0));
    error[2]*=1000;
    error[2]=round(error[2]);
    error[2]/=1000;
    vel_orientation_tool << error[0], error[1], error[2];
    std::cout << "error: " << vel_orientation_tool[0] << " " << vel_orientation_tool[1] << " " << vel_orientation_tool[2] << std::endl;
    vel_orientation_tool << error[0], error[1], 0;
    vel_orientation_tool = tr->rotZ(error[2]).topLeftCorner<3,3>() * vel_orientation_tool;
    vel_orientation_tool[0]  = std::round(vel_orientation_tool[0] * 1000.0) / 1000.0;
    vel_orientation_tool[1]  = std::round(vel_orientation_tool[1] * 1000.0) / 1000.0;
    vel_orientation_tool[2]  = std::round(vel_orientation_tool[2] * 1000.0) / 1000.0;
    std::cout << "error: " << vel_orientation_tool[0] << " " << vel_orientation_tool[1] << " " << vel_orientation_tool[2] << std::endl;
    vel_orientation_base = T_TTP.topLeftCorner<3,3>() * vel_orientation_tool;
    vel_orientation_base[0]  = std::round(vel_orientation_base[0] * 1000.0) / 1000.0;
    vel_orientation_base[1]  = std::round(vel_orientation_base[1] * 1000.0) / 1000.0;
    vel_orientation_base[2]  = std::round(vel_orientation_base[2] * 1000.0) / 1000.0;
    std::cout << "velAngular: " << vel_orientation_base[0] << " " << vel_orientation_base[1] << " " << vel_orientation_base[2] << std::endl;
    //--------------------
    //std::vector<double> velVector(delta_cartesian.data(), delta_cartesian.data() + delta_cartesian.size());
    std::vector<double> velVector = {delta_position[0], delta_position[1], delta_position[2], vel_orientation_base[0], vel_orientation_base[1], 0}; //EN EL QUE FUNCIONA VEL_ORIENTATION_BASE[2]=0 "robotForceControl.cpp"
    //velVector = {delta_position[0], delta_position[1], delta_position[2], 0,0,0,};
    //velVector = {0,0,0, vel_orientation_base[0], vel_orientation_base[1], 0}; //EN EL QUE FUNCIONA VEL_ORIENTATION_BASE[2]=0 "robotForceControl.cpp"
    std::cout << "velVector" << velVector[0] <<"," << velVector[1] << "," << velVector[2] << "," << velVector[3] << "," << velVector[4] << "," << velVector[5] << std::endl;
    ur->speedl(velVector, 0.1, 0.1);
    //Publish vel
    array_vel.data = velVector;
    vel_pub_.publish(array_vel);
    array_vel.data.clear();
    std::cout << "++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++" << std::endl;
}

geometry_msgs::Point jacobianControl::generateNextPoint(const geometry_msgs::Point& desPose, const Eigen::MatrixXd& T_TTP){
    geometry_msgs::Point nextPoint;
    double step_size = 0.01;
    delta_position << (desPose.x - T_TTP(0,3)),
                    (desPose.y - T_TTP(1,3)),
                    (desPose.z - T_TTP(2,3));
    double dist = delta_position.norm();
    if (dist < step_size) {
        nextPoint = desPose;
    } else {
        Eigen::Vector3d delta = step_size *(delta_position / dist); // Vector unitario
        nextPoint.x = T_TTP(0, 3) + delta[0];
        nextPoint.y = T_TTP(1, 3) + delta[1];
        nextPoint.z = T_TTP(2, 3) + delta[2];
    }
    return nextPoint;
}
std::vector<double> jacobianControl::getPolarCoordinates(const geometry_msgs::Point& fulcrum_point_current, double tool_length){
    std::vector<double> polarCoordinates(3);
    geometry_msgs::Point fulcrum_point;
    if (abs(fulcrum_point_current.x) < 0.001){
        fulcrum_point.x = 0;
    } else {
        fulcrum_point.x = fulcrum_point_current.x;
    }
    if (abs(fulcrum_point_current.y) < 0.01){
        fulcrum_point.y = 0;
    } else {
        fulcrum_point.y = fulcrum_point_current.y;
    }
    if (abs(fulcrum_point_current.z) < 0.001){
        fulcrum_point.z = 0;
    } else {
        fulcrum_point.z = fulcrum_point_current.z;
    }
    //current polar coordinates
    polarCoordinates[0] = std::atan2(-fulcrum_point.y,-fulcrum_point.x);
    polarCoordinates[1] = std::atan2(-fulcrum_point.z,sqrt(fulcrum_point.x*fulcrum_point.x+fulcrum_point.y*fulcrum_point.y));
    polarCoordinates[2] = tool_length - sqrt(fulcrum_point.x*fulcrum_point.x+fulcrum_point.y*fulcrum_point.y+fulcrum_point.z*fulcrum_point.z);
    return polarCoordinates;
}

//funciones
void jacobianControl::initializeRobot(int type, double p_estimado_init, double tool_length, std::vector<double> initPosition){
    p_estimado = p_estimado_init;
    std::cout << "vamos a inicializar" << std::endl;
    init->initialize(initPosition,tool0, ur);
    tool->computeTwrist(type, p_estimado_init, tool_length, tr);
    E_T_TTP = tool->E_T_TTP;
    E_T_Fp = tool->E_T_Fp;
    TCP = tool->TCP;//TCP;//DFP
    ur->set_tcp(TCP);
    ros::Duration(0.5).sleep();
}
void jacobianControl::computeRobotCinematic(double L){
    if (prefix_in.empty()) {
        base_name = prefix_in + "base";
        tool_name = prefix_in + "tool0_controller";
        efector_name = prefix_in + "tool0";
    } else {
        base_name = prefix_in + "_base";
        tool_name = prefix_in + "_tool0_controller";
        efector_name = prefix_in + "_tool0";
    }
    T_E = readTransform(base_name, efector_name);
    T_TTP = readTransform(base_name, tool_name);
    if (newFulcrum) fulcrum_position = computeFulcrum(E_T_Fp, T_E); //CAMBIAR ESTO Ñ
    newFulcrum = false;  
    //Current position from Fulcrum
    fulcrum_point_current.x = T_TTP(0,3) - fulcrum_position.x;
    fulcrum_point_current.y = T_TTP(1,3) - fulcrum_position.y;
    fulcrum_point_current.z = T_TTP(2,3) - fulcrum_position.z;
    //Des position from Fulcrum
    // generador trayectoria punto siguiente
    geometry_msgs::Point nextPoint = generateNextPoint(desPose, T_TTP);
    fulcrum_point_des.x = desPose.x- fulcrum_position.x;
    fulcrum_point_des.y = desPose.y - fulcrum_position.y;    
    fulcrum_point_des.z = desPose.z - fulcrum_position.z;
    
    //current polar coordinates
    current_polar_angles = getPolarCoordinates(fulcrum_point_current, L);
    des_polar_angles = getPolarCoordinates(fulcrum_point_des, L);
    if (desPoseReceived) {
        moveToDesirePosition_jacobian(T_TTP, desPose, des_polar_angles, current_polar_angles, L);
    }
    //pub fulcrum_position
    fulcrum_position_.data.clear();
    fulcrum_position_.data.push_back(fulcrum_position.x);
    fulcrum_position_.data.push_back(fulcrum_position.y);
    fulcrum_position_.data.push_back(fulcrum_position.z);
    fulcrum_pub_.publish(fulcrum_position_);
    //pub robot pose T_TTP and T_E
    for (int i = 0; i < T_TTP.rows(); ++i) {
        for (int j = 0; j < T_TTP.cols(); ++j) {
            pose_.data.push_back(T_TTP(j,i));
        }
    }
    ttp_pub_.publish(pose_);
    pose_.data.clear();
    for (int i = 0; i < T_E.rows(); ++i) {
        for (int j = 0; j < T_E.cols(); ++j) {
            poseE_.data.push_back(T_E(j,i));
        }
    }
    te_pub_.publish(poseE_);
    poseE_.data.clear();
    currentPolar_.data.clear();
    //desPolar_.data.clear();
    deltaPolar_.data.clear();
    for (int i = 0; i < current_polar_angles.size(); ++i) {
        currentPolar_.data.push_back(current_polar_angles[i]);
        //desPolar_.data.push_back(des_polar_angles[i]);
        deltaPolar_.data.push_back(delta_polar[i]);
    }
    currentPolar_pub_.publish(currentPolar_);
    //desPolar_pub_.publish(desPolar_);
    deltaPolar_pub_.publish(deltaPolar_);
}
 
int main(int argc, char **argv){
    //ros
    ros::init(argc, argv, "MyJacobianControl");
    ros::NodeHandle nh_param("~"); // Nota: el uso de '~' para obtener los parámetros relativos al namespace del nodo
    ros::Rate rate(10); //frecuencia a cambiar. De normal a 125 Hz
    //launch param
    std::string prefix, sensor_ip;
    double kp, ki, kd, kf;
    double p_estimado_init, tool_length,base, shoulder, elbow, wrist1, wrist2, wrist3;
    int type;
    std::vector<double> initPosition;
    nh_param.param<std::string>("prefix", prefix, "alice");
    nh_param.param<double>("kp", kp, 1);
    nh_param.param<double>("ki", ki, 0);
    nh_param.param<double>("kd", kd, 0);
    nh_param.param<double>("kf", kf, 0);
    nh_param.param<double>("p_estimado", p_estimado_init, 0.1);
    nh_param.param<double>("tool_length", tool_length, 0.2);
    nh_param.param<int>("type", type, 3);
    nh_param.param<std::string>("sensor_ip", sensor_ip, "192.168.1.13");
    nh_param.param<double>("base", base, 90);
    nh_param.param<double>("shoulder", shoulder, 90);
    nh_param.param<double>("elbow", elbow, 90);
    nh_param.param<double>("wrist1", wrist1, 90);
    nh_param.param<double>("wrist2", wrist2, 90);
    nh_param.param<double>("wrist3", wrist3, 0);
    initPosition.push_back(base * DEG_TO_RAD);
    initPosition.push_back(shoulder * DEG_TO_RAD);
    initPosition.push_back(elbow * DEG_TO_RAD);
    initPosition.push_back(wrist1 * DEG_TO_RAD);
    initPosition.push_back(wrist2 * DEG_TO_RAD);
    initPosition.push_back(wrist3 * DEG_TO_RAD);
    ur_script ur(prefix);
    UMA_trans tr;
    Init init;
    PIDController controlPosition(kp,0,kd,0.008,6);
    selectTool tool;
    fulcrum fulcrumEstimation;
    tf::TransformListener tf_listener;
    FTSensor ftSensor(sensor_ip);
    //cosas del sesnor
    std::vector<double> forces;
    if (ftSensor.tareSensor()) {
        std::cout << "Sensor tared successfully\n";
    } else {
        std::cerr << "Failed to tare sensor\n";
    }
    //----------------
    //urJacobian jacob;
    jacobianControl robot(&ur, &tr, &init, &controlPosition, &tool, &fulcrumEstimation, &ftSensor,  &tf_listener,prefix);
    robot.initializeRobot(type, p_estimado_init, tool_length, initPosition);
    ros::Duration(0.5).sleep();
    robot.computeRobotCinematic(tool_length);
    std::cout << "vamos al while" << std::endl;
    while (ros::ok()){
        //robot.computeRobotCinematic(tool_length);
        std::cout << "**********************************************************************\n";
        std::cout << "LECTURA SENSORES  "<<sensor_ip<<"\n";
        if (ftSensor.readFT(forces)) {
            printf("Fx: %.4f N, Fy: %.4f N, Fz: %.4f N, Tx: %.4f Nm, Ty: %.4f Nm, Tz: %.4f Nm\n",
                    forces[0], forces[1], forces[2], forces[3], forces[4], forces[5]);
        } else {
            std::cerr << "Failed to read sensor \n";
        }
        std::cout << "-----------------------------------------------------------------------------\n";
        ros::spinOnce();
        rate.sleep();
    }
}