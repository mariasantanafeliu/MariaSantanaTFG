#include <services/jacobianControl.hpp>
#include <std_msgs/Time.h>

jacobianControl::jacobianControl(ur_script* urScript, UMA_trans* umaTf,
                       Init* initRobot, PIDController* composePID, selectTool* selectTool, fulcrum* fp, FTSensor* ftSensor,  tf::TransformListener* tf_listener, std::string t_prefix) : 
                       ur(urScript), tr(umaTf), init(initRobot), controlPosition(composePID), tool(selectTool), fulcrumEstimation(fp), ftSensor(ftSensor), listener(tf_listener){//lista de inicialización
    std::cout << t_prefix << " control build" << std::endl;
    std::cout << "nh_ ok?: " << nh_.ok() << std::endl;
    prefix_in = t_prefix;
    // Subscripciones a los topics
    goal_pos_sub_ = nh_.subscribe("/coordinator/goal_position", 1000, &jacobianControl::cb_stitchCallback, this);
    fulcrum_sub_ = nh_.subscribe("/coordinator/fulcrum_position", 1000, &jacobianControl::cb_fulcrumCallback, this);
    // Publicacion topics
    ttp_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("pose_topic", 1000);
    te_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("effectorFinal_topic", 1000);
    vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("velocity_topic", 1000);
    currentPolar_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("polar/current", 1000);
    desPolar_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("polar/des", 1000);
    deltaPolar_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("polar/delta", 1000);
    fulcrum_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("fulcrum", 1000);
    force_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("effectorFinal_force", 1000);
    base_force_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("robotbase_force", 1000);
    start_time = ros::Time::now();
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
    /*std::cout << "T_TTP: " << T_TTP << std::endl;
    std::cout << "desPose: " << desPose << std::endl;
    std::cout << "fulcrum_position: " << fulcrum_position << std::endl;*/
}
void jacobianControl::cb_fulcrumCallback(const geometry_msgs::Point::ConstPtr& msg){
    P0[0] = msg->x;
    P0[1] = msg->y;
    P0[2] = (msg->z-0.006) + 0.01; // ❗
    //std::cout << "msg->z: " << msg->z << std::endl;
    //std::cout << "fulcrum: " << P0[2] << std::endl;
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
        return T; 
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
Vector3d intersection(Vector3d P1, Vector3d P2, Vector3d P0) {
    //P0 punto en el plano fulcro
    //P1 TTP
    //P2 TE
    Vector3d ray = P2 - P1;
    //ROS_INFO_STREAM(ray);
    Vector3d normal(0, 0, 1); // plane normal is perpendicular to the z-axis
    // calculate the denominator of the equation for t
    double denominator = normal.dot(ray);
    // check if the denominator is zero (line and plane are parallel)
    if (denominator == 0) {
        return Vector3d(NAN, NAN, NAN);
    }
    double t = (normal.dot(P0) - normal.dot(P1)) / denominator;
    Vector3d P_intersection = P1 + t * ray;
    return P_intersection;
}

void jacobianControl::moveToDesirePosition_jacobian(Eigen::MatrixXd T_TTP, geometry_msgs::Point nextPose, std::vector<double> des_polarCoordinates, std::vector<double> polarCoordinates, double L){
    //get dynamic linear velocity
    delta_position << (nextPose.x - T_TTP(0,3)),
                    (nextPose.y - T_TTP(1,3)),
                    (nextPose.z - T_TTP(2,3));
    delta_position[0]  = std::round(delta_position[0] * 10000.0) / 10000.0;
    delta_position[1]  = std::round(delta_position[1] * 10000.0) / 10000.0;
    delta_position[2]  = std::round(delta_position[2] * 10000.0) / 10000.0;

    std::vector<double> current_angle(3), des_angle(3), error(3);
    Eigen::Vector3d vel_orientation_tool, vel_orientation_base;
    Eigen::MatrixXd R_dest(4,4), dR(4,4), R_current(4,4);
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
    //
    vel_orientation_tool << error[0], error[1], 0;
    vel_orientation_tool = tr->rotZ(error[2]).topLeftCorner<3,3>() * vel_orientation_tool;
    vel_orientation_tool[0]  = std::round(vel_orientation_tool[0] * 1000.0) / 1000.0;
    vel_orientation_tool[1]  = std::round(vel_orientation_tool[1] * 1000.0) / 1000.0;
    vel_orientation_tool[2]  = std::round(vel_orientation_tool[2] * 1000.0) / 1000.0;
    //
    vel_orientation_base = T_TTP.topLeftCorner<3,3>() * vel_orientation_tool;
    vel_orientation_base[0]  = std::round(vel_orientation_base[0] * 1000.0) / 1000.0;
    vel_orientation_base[1]  = std::round(vel_orientation_base[1] * 1000.0) / 1000.0;
    vel_orientation_base[2]  = std::round(vel_orientation_base[2] * 1000.0) / 1000.0;
    //--------------------
    std::vector<double> velVector = {delta_position[0], delta_position[1], delta_position[2], vel_orientation_base[0], vel_orientation_base[1], 0};
    //std::vector<double> velVector = {delta_position[0], delta_position[1], delta_position[2], 0,0, 0};
    ur->speedl(velVector, 0.1, 0.1);
    //Publish vel
    array_vel.data = velVector;
    vel_pub_.publish(array_vel);
    array_vel.data.clear();
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
    if (newFulcrum){
        Vector3d P2;// = T_E.block(0, 2, 3, 1);
        P2[0]=T_E(0,3);
        P2[1]=T_E(1,3);
        P2[2]=T_E(2,3);
        Vector3d P1;// = T_TTP.block(0, 2, 3, 1);
        P1[0]=T_TTP(0,3);
        P1[1]=T_TTP(1,3);
        P1[2]=T_TTP(2,3);
        Vector3d P_intersection = intersection(P1, P2, P0);
        fulcrum_position.x = P_intersection[0];
        fulcrum_position.y = P_intersection[1];
        fulcrum_position.z = P0[2];
        //std::cout << "fulcrum_position: " << fulcrum_position << std::endl;
    }  
    //forces
    ftSensor->readFT(forces);
    robotBaseForce = T_E.topLeftCorner<3,3>() * Eigen::Vector3d(forces[0], forces[1], forces[2]);
    baseForces = {robotBaseForce(0), robotBaseForce(1), robotBaseForce(2)};
    //Current position from Fulcrum
    fulcrum_point_current.x = T_TTP(0,3) - fulcrum_position.x;
    fulcrum_point_current.y = T_TTP(1,3) - fulcrum_position.y;
    fulcrum_point_current.z = T_TTP(2,3) - fulcrum_position.z;
    //
    fulcrum_point_des.x = desPose.x- fulcrum_position.x;
    fulcrum_point_des.y = desPose.y - fulcrum_position.y;    
    fulcrum_point_des.z = desPose.z - fulcrum_position.z;
    
    //current polar coordinates
    current_polar_angles = getPolarCoordinates(fulcrum_point_current, L);
    des_polar_angles = getPolarCoordinates(fulcrum_point_des, L);
    if (desPoseReceived && (abs(forces[2]) < 25)) {
        newFulcrum = false;
        moveToDesirePosition_jacobian(T_TTP, desPose, des_polar_angles, current_polar_angles, L);
    } else{
        newFulcrum = true;
        array_vel.data = {0,0,0,0,0,0};
        vel_pub_.publish(array_vel);
        array_vel.data.clear();
    }
    //pub forces
    force_msg_.data.clear();
    force_msg_.data.push_back(forces[0]);
    force_msg_.data.push_back(forces[1]);
    force_msg_.data.push_back(forces[2]);
    force_msg_.data.push_back(forces[3]);
    force_msg_.data.push_back(forces[4]);
    force_msg_.data.push_back(forces[5]);
    force_pub_.publish(force_msg_);
    base_force_msg_.data.clear();
    base_force_msg_.data.push_back(baseForces[0]);
    base_force_msg_.data.push_back(baseForces[1]);
    base_force_msg_.data.push_back(baseForces[2]);
    base_force_pub_.publish(base_force_msg_);
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

/*ros::Time jacobianControl::getStartTime() const {
    return start_time;
}*/
 
int main(int argc, char **argv){
    //ros
    ros::init(argc, argv, "MyJacobianControl");
    ros::NodeHandle nh_param("~"); // Nota: el uso de '~' para obtener los parámetros relativos al namespace del nodo
    ros::Rate rate(125); //frecuencia a cambiar. De normal a 125 Hz
    ros::Publisher tissue_force_pub_ = nh_param.advertise<std_msgs::Float64MultiArray>("tissue_force", 1000);
    std_msgs::Float64MultiArray tissue_force_msg;
    //launch param
    std::string prefix, sensor_ip;
    double kp, ki, kd, kf;
    double p_estimado_init, tool_length,base, shoulder, elbow, wrist1, wrist2, wrist3;
    int type;
    std::vector<double> initPosition;
    nh_param.param<std::string>("prefix", prefix, "auto");
    nh_param.param<double>("tool_length", tool_length, 0.2);
    nh_param.param<int>("type", type, 1);
    nh_param.param<double>("p_estimado", p_estimado_init, 0.1);
    nh_param.param<std::string>("sensor_ip", sensor_ip, "192.168.1.1");
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
    PIDController controlPosition(0,0,0,0.008,6);
    selectTool tool;
    fulcrum fulcrumEstimation;
    FTSensor ftSensor("192.168.1.1");
    FTSensor tissueSensor("192.168.1.13");
    tf::TransformListener tf_listener;
    //cosas del sesnor
    std::vector<double> forces, tissueForces;
    if (ftSensor.tareSensor()) {
        std::cout << "Sensor tared successfully\n";
    } else {
        std::cerr << "Failed to tare sensor\n";
    }
    if (tissueSensor.tareSensor()) {
        std::cout << "Sensor tared successfully\n";
    } else {
        std::cerr << "Failed to tare sensor\n";
    }
    //----------------
    //urJacobian jacob;
    jacobianControl robot(&ur, &tr, &init, &controlPosition, &tool, &fulcrumEstimation, &ftSensor, &tf_listener,prefix);
    robot.initializeRobot(type, 0, tool_length, initPosition);
    ros::Duration(1).sleep();
    //ros::Time start_time = robot.getStartTime();
    robot.computeRobotCinematic(tool_length);
    //time

    //
    std::cout << "vamos al while" << std::endl;
    while (ros::ok()){
        robot.computeRobotCinematic(tool_length);
        if (tissueSensor.readFT(tissueForces)) {
            //printf("Fx: %.4f N, Fy: %.4f N, Fz: %.4f N, Tx: %.4f Nm, Ty: %.4f Nm, Tz: %.4f Nm\n",
                    //tissueForces[0], tissueForces[1], tissueForces[2], tissueForces[3], tissueForces[4], tissueForces[5]);
        } else {
            std::cerr << "Failed to read sensor \n";
            tissueForces = {0,0,0,0,0,0};
        }
        tissue_force_msg.data.clear();
        tissue_force_msg.data.push_back(tissueForces[0]);
        tissue_force_msg.data.push_back(tissueForces[1]);
        tissue_force_msg.data.push_back(tissueForces[2]);
        tissue_force_msg.data.push_back(tissueForces[3]);
        tissue_force_msg.data.push_back(tissueForces[4]);
        tissue_force_msg.data.push_back(tissueForces[5]);
        tissue_force_pub_.publish(tissue_force_msg);
        ros::spinOnce();
        rate.sleep();
    }
}