#include <services/jacobianControl.hpp>

jacobianControl::jacobianControl(ur_script* urScript, UMA_trans* umaTf,
                       Init* initRobot, PIDController* composePID, selectTool* selectTool, fulcrum* fp, FTSensor* ftSensor,  tf::TransformListener* tf_listener, std::string t_prefix) : 
                       ur(urScript), tr(umaTf), init(initRobot), controlPosition(composePID), tool(selectTool), fulcrumEstimation(fp), ftSensor(ftSensor), listener(tf_listener){//lista de inicialización
    std::cout << t_prefix << " control build" << std::endl;
    prefix_in = t_prefix;
    // Subscripciones a los topics
    force_sensor_sub_ = nh_.subscribe("/ur3e/rtde/force", 1000, &jacobianControl::cb_readForceTorque, this);
    // Publicacion topics
    ttp_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("pose_topic", 1000);
    vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("velocity_topic", 1000);
    te_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("effectorFinal_topic", 1000);
    reset_sensor_pub_ = nh_.advertise<std_msgs::String>("/"+t_prefix+"/ur_hardware_interface/script_command",1000);
    abdomen_force_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("abdomen_force_topic", 1000);
    abdomen_effector_force_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("abdomen_effectorForce_topic", 1000);
}
jacobianControl::~jacobianControl(){
    std::cout <<"Leaving gently ur control..."<< std::endl;
}

void jacobianControl::cb_readForceTorque(const geometry_msgs::Twist::ConstPtr& msg){
    //reset sensor
    if(first_force_reading){
        std_msgs::String reset;
        reset.data = "zero_ftsensor()";
        reset_sensor_pub_.publish(reset);
        first_force_reading = false;
    }
    // fuerza base robot
    forceRobotBase << msg->linear.x, msg->linear.y, msg->linear.z;
    torqueRobotBase << msg->angular.x, msg->angular.y, msg->angular.z;
    // fuerza efector final robot
    Eigen::Matrix3d R = T_E.topLeftCorner<3,3>(); 
    forceRobotTTP = R.inverse()*forceRobotBase;
    torqueRobotTTP = R.inverse()*torqueRobotBase; 
}
/*void jacobianControl::cb_readForceTorque(const geometry_msgs::Twist::ConstPtr& msg){
    forceRobotBase << msg->linear.x, msg->linear.y, msg->linear.z;
    torqueRobotBase << msg->angular.x, msg->angular.y, msg->angular.z;
    // fuerza efector final robot
    Eigen::Matrix3d R = T_TTP.topLeftCorner<3,3>();
    forceRobotTTP = R.inverse()*forceRobotBase;
    torqueRobotTTP = R.inverse()*torqueRobotBase;
}*/

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
    //
    abdomen_force_msg_.data.clear();
    abdomen_force_msg_.data.push_back(forceRobotBase[0]);
    abdomen_force_msg_.data.push_back(forceRobotBase[1]);
    abdomen_force_msg_.data.push_back(forceRobotBase[2]);
    abdomen_force_msg_.data.push_back(torqueRobotBase[0]);
    abdomen_force_msg_.data.push_back(torqueRobotBase[1]);
    abdomen_force_msg_.data.push_back(torqueRobotBase[2]);
    abdomen_force_pub_.publish(abdomen_force_msg_);
    //
    abdomen_force_effector_msg_.data.clear();
    abdomen_force_effector_msg_.data.push_back(forceRobotTTP[0]);
    abdomen_force_effector_msg_.data.push_back(forceRobotTTP[1]);
    abdomen_force_effector_msg_.data.push_back(forceRobotTTP[2]);
    abdomen_force_effector_msg_.data.push_back(torqueRobotTTP[0]);
    abdomen_force_effector_msg_.data.push_back(torqueRobotTTP[1]);
    abdomen_force_effector_msg_.data.push_back(torqueRobotTTP[2]);
    abdomen_effector_force_pub_.publish(abdomen_force_effector_msg_);
}
int main(int argc, char **argv){
    //ros
    ros::init(argc, argv, "MyControlForce");
    ros::NodeHandle nh_param("~"); // Nota: el uso de '~' para obtener los parámetros relativos al namespace del nodo
    ros::Rate rate(125); //frecuencia a cambiar. De normal a 125 Hz
    //launch param
    std::string prefix;
    double tool_length;
    nh_param.param<std::string>("prefix", prefix, "darel");
    nh_param.param<double>("tool_length", tool_length, 0.2);
    ur_script ur(prefix);
    UMA_trans tr;
    Init init;
    PIDController controlPosition(0,0,0,0.008,6);
    selectTool tool;
    fulcrum fulcrumEstimation;
    FTSensor ftSensor("192.168.1.1");
    tf::TransformListener tf_listener;
    jacobianControl robot(&ur, &tr, &init, &controlPosition, &tool, &fulcrumEstimation, &ftSensor, &tf_listener,prefix);
    //----------------
    std::vector<double> TCP = {0., 0, tool_length, 0., 0., 0.};
    ur.set_tcp(TCP);
    ros::Duration(1).sleep();
    robot.computeRobotCinematic(tool_length);
    std::cout << "vamos al while obstaculo" << std::endl;
    double i = 0.0;
    while (ros::ok()){
        robot.computeRobotCinematic(tool_length);
        ros::spinOnce();
        rate.sleep();
    }
}