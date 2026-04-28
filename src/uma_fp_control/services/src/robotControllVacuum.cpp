#include <services/robotControl.hpp>

ur3Control::ur3Control(ur_script* urScript, UMA_trans* umaTf, PoseDesJM* estimatePose,
                       Init* initRobot, ErrorPose* composeError, forceControl* composeforceControl, 
                       PIDController* composePID, Cinematic* robotTransf, selectTool* selectTool, std::string t_prefix) : 
                       ur(urScript), tr(umaTf), Pdest(estimatePose),
                       init(initRobot), error(composeError), Kforce(composeforceControl), 
                       controlPosition(composePID), T(robotTransf), tool(selectTool){//lista de inicialización
    std::cout << t_prefix << " control build" << std::endl;
    prefix_in = t_prefix;
    // Subscripciones a los topics
    goal_pos_sub_ = nh_.subscribe("/"+t_prefix+"/coordinator/goal_position", 1000, &ur3Control::cb_stitchCallback, this);
    //force_sensor_sub_ = nh_.subscribe("/"+t_prefix + "/sensor", 1000, &ur3Control::cb_readForceTorque, this);//"/HEX_70_XE_1/"
    //surface_sub_ = nh_.subscribe("/sensor", 1000, &ur3Control::cb_readSurface, this);
    // Publicacion topics
    ttp_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("pose_topic", 1000);
    error_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("error_topic", 1000);
    ///aspirator/led
    last_desPose.x = 0;
    last_desPose.y = 0;
    last_desPose.z = 0;
    desPose.x = 0;
    desPose.y = 0;
    desPose.z = 0;
}
ur3Control::~ur3Control(){
    std::cout <<"Leaving gently ur control..."<< std::endl;
}
//#######################callback##########################
void ur3Control::cb_stitchCallback(const geometry_msgs::Point::ConstPtr& msg){
    desPose.x = msg->x;
    desPose.y = msg->y;
    desPose.z = msg->z;
    std::cout << "... cb_stitchCallback VACUUM" << std::endl;
    std::cout << desPose << std::endl;
}

//#######################function##########################
Eigen::MatrixXd ur3Control::readTransform(std::string base,std::string tool0){
    Eigen::MatrixXd T(4,4);
    try{
        listener.lookupTransform(base, tool0, ros::Time(0), tf_pose);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
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
void ur3Control::initializeRobot(int type, double p_estimado_init, std::vector<double> initPosition){
    p_estimado = p_estimado_init;
    init->initialize(initPosition,tool0, ur);
    tool->computeTwrist(type, p_estimado_init, 0.301, tr);
    E_T_TTP = tool->E_T_TTP;
    E_T_Fp = tool->E_T_Fp;
    TCP = tool->TCP;
    std::cout << "... TCP= " <<TCP[2] << std::endl;
    ur->set_tcp(TCP);
    ros::Duration(0.5).sleep();
}

void ur3Control::computeRobotCinematic(bool flagFp){
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
    T->computeCinematicTf(TTP_dest, T_E, T_TTP, desPose, last_desPose, prefix_in, E_T_TTP, E_T_Fp ,T_Fp, p_estimado,flagFp,fulcrumForces, tr, Pdest);
    TTP_dest = T->TTP_dest_orientado;
    T_TTP = T->T_TTP;
    /*std::cout << "----------------------------------------------------------------------------------------------------------------------" << std::endl;
    std::cout << "T_E" << std::endl;
    std::cout << T_E << std::endl;
    std::cout << "T_TTP" << std::endl;
    std::cout << T_TTP << std::endl;
    std::cout << "TTP_dest" << std::endl;
    std::cout << TTP_dest << std::endl;*/
    T_Fp = T->T_Fp;
    T_FpOG = T->T_Fp;
   /*std::cout << "T_Fp" << std::endl;
    std::cout << T_FpOG << std::endl;
    std::cout << "T_FpOG" << std::endl;*/
    last_desPose = desPose;
    for (int i = 0; i < T_TTP.rows(); ++i) {
        for (int j = 0; j < T_TTP.cols(); ++j) {
            pose_.data.push_back(T_TTP(j,i));
        }
    }
    ttp_pub_.publish(pose_);
    pose_.data.clear();
}
void ur3Control::computePoseError(){
    diffPose = error->computeErrorTf(TTP_dest, T_TTP);
    for (int i = 0; i < 6; ++i) {
        error_.data.push_back(diffPose[i]);
    }
    //std::cout << " error_r c= " << error_ << std::endl;
    error_pub_.publish(error_);
    error_.data.clear();
}
void ur3Control::computeRobotVel(){
    velVector = controlPosition->calculate(diffPose);
    //std::cout << " diffPose= " << diffPose[0] << ",  "<< diffPose[1] << ",  "<< diffPose[2] << ",  "<< diffPose[3] << ",  "<< diffPose[4] << std::endl;
    velLineal << diffPose[0] ,diffPose[1],diffPose[2],1;
    velAngular << diffPose[3], diffPose[4],diffPose[5],0;
    //transformadas
    //velLineal << 0,velLineal(1),0,0;
    //velAngular << velAngular(0),0,0,0;
    velAngular = T_TTP * velAngular; //la diferencia de posicion se hace respecto a la base, PERO el de angulo no, por eso esto es necesario POR USAR dR
    velVector = {velLineal(0),velLineal(1),velLineal(2),velAngular(0),velAngular(1),0};
    //velVector = {velLineal(0),velLineal(1),velLineal(2),0,0,0};
    //send velocity to robot
    //std::cout << " velVector2= " << velVector[0] << ",  "<< velVector[1] << ",  "<< velVector[2] << ",  "<< velVector[3] << ",  "<< velVector[4] << std::endl;
    ur->speedl(velVector, 0.1, 1/125);
}   
int main(int argc, char **argv){
    //ros
    ros::init(argc, argv, "Vacuum");
    ros::NodeHandle nh_param("~"); // Nota el uso de '~' para obtener los parámetros relativos al namespace del nodo
    ros::Rate rate(125);
    //launch param
    std::string prefix;
    double kp, ki, kd, kf;
    double p_estimado_init, tool_length ,base, shoulder, elbow, wrist1, wrist2, wrist3;
    int type;
    std::vector<double> initPosition;
    nh_param.param<std::string>("prefix", prefix, "auto");
    nh_param.param<double>("kp", kp, 1);
    nh_param.param<double>("ki", ki, 0);
    nh_param.param<double>("kd", kd, 0);
    nh_param.param<double>("kf", kf, 0);
    nh_param.param<double>("p_estimado", p_estimado_init, 0.1);
    nh_param.param<double>("tool_length", tool_length, 0.2);
    nh_param.param<int>("type", type, 3);
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
    PoseDesJM Pdest;
    Init init;
    ErrorPose error;
    forceControl Kforce;
    PIDController controlPosition(kp,ki,kd,1/125,6); //(double kp, double ki, double kd, double time_interval, int num_controllers)
    Cinematic T;
    selectTool tool;
    ur3Control robot(&ur, &tr, &Pdest, &init, &error, &Kforce, &controlPosition, &T, &tool, prefix);
    robot.initializeRobot(type, p_estimado_init, initPosition);
    ros::Duration(0.5).sleep();
    robot.computeRobotCinematic(true);
    std::cout << "vamos al while" << std::endl;
    while (ros::ok()){
        robot.computeRobotCinematic(false);
        robot.computePoseError();
        robot.computeRobotVel();
        ros::spinOnce();
        rate.sleep();
    }
}