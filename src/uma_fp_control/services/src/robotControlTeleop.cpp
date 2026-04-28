#include <services/robotControl.hpp>

ur3Control::ur3Control(ur_script* urScript, UMA_trans* umaTf, PoseDesJM* estimatePose,
                       Init* initRobot, ErrorPose* composeError, forceControl* composeforceControl, 
                       PIDController* composePID, Cinematic* robotTransf, selectTool* selectTool,  tf::TransformListener* tf_listener,std::string t_prefix) : 
                       ur(urScript), tr(umaTf), Pdest(estimatePose),
                       init(initRobot), error(composeError), Kforce(composeforceControl), 
                       controlPosition(composePID), T(robotTransf), tool(selectTool), listener(tf_listener){//lista de inicialización
    std::cout << t_prefix << " control build" << std::endl;
    prefix_in = t_prefix;
    // Subscripciones a los topics
    goal_pos_sub_ = nh_.subscribe("/"+t_prefix+"/coordinator/goal_position", 1000, &ur3Control::cb_stitchCallback, this);
    //force_sensor_sub_ = nh_.subscribe("/"+t_prefix + "/sensor", 1000, &ur3Control::cb_readForceTorque, this);//"/HEX_70_XE_1/"
    //surface_sub_ = nh_.subscribe("/sensor", 1000, &ur3Control::cb_readSurface, this);
    // Publicacion topics
    ttp_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("pose_topic", 1000);
    error_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("error_topic", 1000);
    last_desPose.x = 0;
    last_desPose.y = 0;
    last_desPose.z = 0;
    desPose.x = 0;
    desPose.y = 0;
    desPose.z = 0;
    transformRobotbaseToHaptic = tr->rotZ(0);//tr->rotZ(3*3.14/4);
    hapticDisplacement.resize(4, 1);
    robotbaseDisplacement.resize(4, 1);
    hapticRotation.resize(4, 1);
    robotbaseRotation.resize(4, 1);
}
ur3Control::~ur3Control(){
    std::cout <<"Leaving gently ur control..."<< std::endl;
}
//#######################callback##########################
void ur3Control::cb_stitchCallback(const geometry_msgs::Point::ConstPtr& msg){
    desPose.x = msg->x;
    desPose.y = msg->y;
    desPose.z = msg->z;
    //std::cout << "... cb_stitchCallback TELEOP" << std::endl;
}

//#######################function##########################
Eigen::MatrixXd ur3Control::readTransform(std::string base,std::string tool0){
    Eigen::MatrixXd T(4,4);
    try{
        listener->waitForTransform(base, tool0, ros::Time(0), ros::Duration(1.0));
        listener->lookupTransform(base, tool0, ros::Time(0), tf_pose);
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
    tool->computeTwrist(type, p_estimado_init, 4, tr);
    E_T_TTP = tool->E_T_TTP;
    E_T_Fp = tool->E_T_Fp;
    TCP = tool->DFP; //AQUI ME QUEDO CON EL DFP PORQUE ES TELEOP CON CRM COINCIDENTE
    //std::cout << "... DFP= " <<TCP[2] << std::endl;
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
    //*****PUB*/
    for (int i = 0; i < T_TTP.rows(); ++i) {
        for (int j = 0; j < T_TTP.cols(); ++j) {
            pose_.data.push_back(T_TTP(j,i));
        }
    }
    ttp_pub_.publish(pose_);
    pose_.data.clear();
    //std::cout << "salimos de computeRobotCinematic" << std::endl;
}

void ur3Control::computeRobotVel(){
    //Robot teleoperado SOLO MOVER XY
    // Transforms movement from haptic's POV to robot base's POV
    hapticDisplacement << 0,0,-desPose.z*0.4, 0;
    //std::cout << "hapticDisplacement" << hapticDisplacement<< std::endl;
    robotbaseDisplacement = T_TTP * hapticDisplacement;
    deltaP[0] = desPose.x;//robotbaseDisplacement(0);
    deltaP[1] = desPose.y;//robotbaseDisplacement(1);
    deltaP[2] = 0;//robotbaseDisplacement(2);
    hapticRotation << desPose.x * 4, desPose.y * 4,0,0;
    robotbaseRotation = transformRobotbaseToHaptic * hapticRotation;
    /*velVector = {
        robotbaseDisplacement(0, 0),
        robotbaseDisplacement(1, 0),
        robotbaseDisplacement(2, 0),
        robotbaseRotation(0, 0),//deltaP[0]*4,
        robotbaseRotation(1, 0),//deltaP[1]*4, //
        0
    };*/
    hapticRotation << desPose.x, desPose.y,desPose.z,0;
    robotbaseRotation = transformRobotbaseToHaptic * hapticRotation;
    velVector = {
        robotbaseRotation(0, 0),
        robotbaseRotation(1, 0),
        robotbaseRotation(2, 0),
        0,//deltaP[0]*4,
        0,//deltaP[1]*4, //
        0
    };
    //send velocity to robot
    ur->speedl(velVector, 0.1, 1/125);
    //std::cout << "salimos de computeRobotVel" << std::endl;
}   
int main(int argc, char **argv){
    //ros
    ros::init(argc, argv, "teleoperator");
    ros::NodeHandle nh_param("~"); // Nota el uso de '~' para obtener los parámetros relativos al namespace del nodo
    ros::Rate rate(125);
    //pub
    //ros::Publisher force_pub = nh_param.advertise<omni_msgs::OmniFeedback>("/AZUL/phantom/force_feedback", 1);
    //launch param
    std::string prefix;
    double kp, ki, kd, kf;
    double p_estimado_init, tool_length ,base, shoulder, elbow, wrist1, wrist2, wrist3;
    int type;
    std::vector<double> initPosition;
    nh_param.param<std::string>("prefix", prefix, "teleop");
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
    PIDController controlPosition(1,0,0,0.008,6); //(double kp, double ki, double kd, double time_interval, int num_controllers)
    Cinematic T;
    selectTool tool;
    tf::TransformListener tf_listener;
    
    /*FTSensor ftSensor("192.168.1.1");

    std::vector<double> forces;
    if (ftSensor.tareSensor()) {
        std::cout << "Sensor tared successfully\n";
    } else {
        std::cerr << "Failed to tare sensor\n";
    }
    // Crear el mensaje
    omni_msgs::OmniFeedback feedback_msg;
    // Si no necesitas posición, déjala en 0
    feedback_msg.position.x = 0.0;
    feedback_msg.position.y = 0.0;
    feedback_msg.position.z = 0.0;*/
    
    ur3Control robot(&ur, &tr, &Pdest, &init, &error, &Kforce, &controlPosition, &T, &tool ,  &tf_listener, prefix);
    robot.initializeRobot(type, p_estimado_init, initPosition);
    ros::Duration(0.5).sleep();
    robot.computeRobotCinematic(true);
    std::cout << "vamos al while" << std::endl;
    while (ros::ok()){
        /*if (ftSensor.readFT(forces)) {
            //printf("Fx: %.4f N, Fy: %.4f N, Fz: %.4f N, Tx: %.4f Nm, Ty: %.4f Nm, Tz: %.4f Nm\n",
            //        forces[0], forces[1], forces[2], forces[3], forces[4], forces[5]);
        } else {
            std::cerr << "Failed to read sensor \n";
        }

        if (std::fabs(forces[0]) > 0.9 || std::fabs(forces[1]) > 0.9 || std::fabs(forces[2]) > 0.9){
            feedback_msg.force.x = forces[0] / 1;
            feedback_msg.force.y = forces[1] / 1;
            feedback_msg.force.z = forces[2] / 1;
        }
        else{
            feedback_msg.force.x = 0;
            feedback_msg.force.y = 0;
            feedback_msg.force.z = 0;
        }*/
        //force_pub.publish(feedback_msg);

        robot.computeRobotCinematic(false);
        robot.computeRobotVel();
        ros::spinOnce();
        rate.sleep();
    }
}