#include <services/tfg.hpp>
#include <std_msgs/Time.h>

tfg::tfg(ur_script* urScript, UMA_trans* umaTf,
                       Init* initRobot, PIDController* composePID, selectTool* selectTool, fulcrum* fp, FTSensor* ftSensor,  tf::TransformListener* tf_listener, ErrorPose* composeError,std::string t_prefix) : 
                       ur(urScript), tr(umaTf), init(initRobot), controlPosition(composePID), tool(selectTool), fulcrumEstimation(fp), ftSensor(ftSensor), listener(tf_listener){//lista de inicialización
    std::cout << t_prefix << " control build" << std::endl;
    prefix_in = t_prefix;
    // Subscripciones a los topics
    goal_pos_sub_ = nh_.subscribe("/coordinator/goal_position", 1000, &tfg::cb_stitchCallback, this);
    fulcrum_sub_ = nh_.subscribe("/coordinator/fulcrum_position", 1000, &tfg::cb_fulcrumCallback, this);
    // Publicacion topics
    ttp_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("pose_topic", 1000);
    te_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("effectorFinal_topic", 1000);
    vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("velocity_topic", 1000);
    fulcrum_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("fulcrum", 1000);
    force_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("effectorFinal_force", 1000);
    base_force_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("robotbase_force", 1000);

    desPose.x = 0.;
    desPose.y = 0.;
    desPose.z = 0.;
    desPoseReceived = false;//aa
    start_time = ros::Time::now();

    vX = {0.0,0.0,0.0};
    vY = {0.0,0.0,0.0};
    vZ = {0.0,0.0,0.0};
}
tfg::~tfg(){
    std::cout <<"Leaving gently tfg..."<< std::endl;
}
//#######################callback##########################
void tfg::cb_stitchCallback(const geometry_msgs::Point::ConstPtr& msg){
    desPose.x = msg->x;
    desPose.y = msg->y;
    desPose.z = msg->z;
    desPoseReceived = true;
    std::cout <<"desPose-->"<< desPose << std::endl;
    std::cout <<"Leaving gently cb_stitchCallback..."<< std::endl;
}
void tfg::cb_fulcrumCallback(const geometry_msgs::Point::ConstPtr& msg){
    P0[0] = msg->x;
    P0[1] = msg->y;
    P0[2] = (msg->z-0.00);// - 0.02; // ❗
}
//#######################function##########################
Eigen::MatrixXd tfg::readTransform(std::string base,std::string tool0){
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

geometry_msgs::Point tfg::computeFulcrum(Eigen::MatrixXd E_T_Fp, Eigen::MatrixXd T_E){
    geometry_msgs::Point fulcrum_position;
    Eigen::MatrixXd T(4,4);
    T = T_E * E_T_Fp;
    fulcrum_position.x = T(0,3);
    fulcrum_position.y = T(1,3);
    fulcrum_position.z = T(2,3);
    return fulcrum_position;
}

//funciones
void tfg::initializeRobot(int type, double p_estimado_init, double tool_length, std::vector<double> initPosition){
    p_estimado = p_estimado_init;
    std::cout << "vamos a inicializar" << std::endl;
    init->initialize(initPosition,tool0, ur);
    tool->computeTwrist(type, p_estimado_init, tool_length, tr);
    TCP = tool->TCP;//TCP;//DFP
    E_T_TTP = tool->E_T_TTP;
    E_T_Fp = tool->E_T_Fp;
    ur->set_tcp(TCP);
    ros::Duration(0.5).sleep();
    std::cout << "volvemos de inicializar tfg" << std::endl;
}
void tfg::computeRobotCinematic(double L){
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
    //forces
    ftSensor->readFT(forces);
    robotBaseForce = T_E.topLeftCorner<3,3>() * Eigen::Vector3d(forces[0], forces[1], forces[2]);
    baseForces = {robotBaseForce(0), robotBaseForce(1), robotBaseForce(2)};
    //fulcrum
    /*if (newFulcrum){
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
    }*/
    if (newFulcrum) fulcrum_position = computeFulcrum(E_T_Fp, T_E);
    newFulcrum = false;
    //
    //Des position from Fulcrum
    fulcrum_point_des.x = desPose.x - fulcrum_position.x;
    fulcrum_point_des.y = desPose.y - fulcrum_position.y;    
    fulcrum_point_des.z = desPose.z - fulcrum_position.z;
    if (desPoseReceived) {// && (abs(forces[2]) < 20)) {
        
        Eigen::MatrixXd R_Fp(4,4);
        R_Fp = tr->desp({fulcrum_position.x, fulcrum_position.y, fulcrum_position.z});
        vZ[0]=fulcrum_point_des.x;
        vZ[1]=fulcrum_point_des.y;
        vZ[2]=fulcrum_point_des.z;
        auto M_vZ = sqrt(pow(vZ[0],2) + pow(vZ[1],2) + pow(vZ[2],2));
        vZ[0]= vZ[0] / M_vZ;
        vZ[1]= vZ[1] / M_vZ;
        vZ[2]= vZ[2] / M_vZ;
        vX[0]= vZ[2];
        vX[1]= 0;
        vX[2]= -vZ[0];
        auto M_vX = sqrt(pow(vX[0],2) + pow(vX[1],2) + pow(vX[2],2));
        vX[0]= vX[0] / M_vX;
        vX[1]= vX[1] / M_vX;
        vX[2]= vX[2] / M_vX;

        vY[0]= vZ[1]*vX[2]-vZ[2]*vX[1];
        vY[1]= vZ[2]*vX[0]-vZ[0]*vX[2];
        vY[2]= vZ[0]*vX[1]-vZ[1]*vX[0];
        auto M_vY = sqrt(pow(vY[0],2) + pow(vY[1],2) + pow(vY[2],2));
        vY[0]= vY[0] / M_vY;
        vY[1]= vY[1] / M_vY;
        vY[2]= vY[2] / M_vY;
        Eigen::MatrixXd W_Rdest(4,4);
        W_Rdest << vX[0],vY[0],vZ[0],0,vX[1],vY[1],vZ[1],0,vX[2],vY[2],vZ[2],0,0,0,0,1;
        T_dest = R_Fp * W_Rdest;
        
        //T_dest = T_TTP;
        T_dest(0,3) = desPose.x;
        T_dest(1,3) = desPose.y;
        T_dest(2,3) = desPose.z;
        /*std::cout << "T_E=[" << T_E << "]"<< std::endl;
        std::cout << "fulcrum_position=[" << fulcrum_position << "]"<< std::endl;
        std::cout << "T_TTP=[" << T_TTP << "]"<< std::endl;
        std::cout << "T_dest=[" << T_dest << "]"<< std::endl;*/
        velVector[0] = T_dest(0,3) - T_TTP(0,3);
        velVector[0]*=1000;
        velVector[0]=round( velVector[0]);
        velVector[0]/=1000;
        velVector[0]=velVector[0];
        velVector[1] = T_dest(1,3) - T_TTP(1,3);
        velVector[1]*=1000;
        velVector[1]=round( velVector[1]);
        velVector[1]/=1000;
        velVector[1]=velVector[1];
        velVector[2] = T_dest(2,3) - T_TTP(2,3);
        velVector[2]*=1000;
        velVector[2]=round( velVector[2]);
        velVector[2]/=1000;
        velVector[2]=velVector[2];

        auto dR = T_TTP.transpose() * T_dest; //Con dR se calcula la diferencia de orientacion en EJES moviles. Es decir, en TTP. Por eso luego tenemos que pasar esa diferencia a la base del robot.
        velVector[3] = atan2(dR(2,1), dR(2,2));
        velVector[3]*=1000;
        velVector[3]=round(velVector[3]);
        velVector[3]/=1000;
        velVector[4] = std::atan2(-dR(2,0),sqrt(pow(dR(0,0),2)+pow(dR(1,0),2)));
        velVector[4]*=1000;
        velVector[4]=round(velVector[4]);
        velVector[4]/=1000;
        //error yaw
        velVector[5] = std::atan2(dR(1,0),dR(0,0)); 
        velVector[5]*=1000;
        velVector[5]=round(velVector[5]);
        velVector[5]/=1000;
        velLineal << velVector[0] ,velVector[1],velVector[2],0;
        velAngular << velVector[3], velVector[4],velVector[5],0;
        velVector = {velLineal(0),velLineal(1),velLineal(2),-velAngular(0),velAngular(1),0}; //velVector = {velLineal(0),velLineal(1),velLineal(2),-velAngular(0),velAngular(1),0};
        //std::cout << "velVector: " << velVector[0] << ", " << velVector[1] << ", " << velVector[2] << ", " << velVector[3] << ", " << velVector[4] << ", " << velVector[5] << std::endl;
        
        if ((ros::Time::now() - start_time).toSec() > 5.0){
            ur->speedl(velVector, 0.5, 0.5);
            
            // arreglo problema velocidad
            array_vel.data.clear();
            for (int i = 0; i < 6; ++i) {
                array_vel.data.push_back(velVector[i]);
            }
            vel_pub_.publish(array_vel);
            array_vel.data.clear();
        } else {
            ur->stopl(1);
            std__cout << "[INFO] Posición recibida, pero movimiento bloqueado." << std::endl;
        }
    } else{
        /*std::cout << "fulcrum_point_des=[" << fulcrum_point_des << "]" << std::endl;
        std::cout << "T_E=[" << T_E << "]"<< std::endl;
        std::cout << "T_TTP=[" << T_TTP << "]"<< std::endl;*/
        newFulcrum = true;
        array_vel.data = {0,0,0,0,0,0};
        vel_pub_.publish(array_vel);
        array_vel.data.clear();
    }
    //pub forces
    force_msg_.data.clear();
    if (forces.size() >= 6){
        force_msg_.data.push_back(forces[0]);
        force_msg_.data.push_back(forces[1]);
        force_msg_.data.push_back(forces[2]);
        force_msg_.data.push_back(forces[3]);
        force_msg_.data.push_back(forces[4]);
        force_msg_.data.push_back(forces[5]);
        force_pub_.publish(force_msg_);
    }else {
        std::cout << "Sensor vacio" << forces.size() << std::endl;
    }
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
}
 
int main(int argc, char **argv){
    //ros
    ros::init(argc, argv, "tfg");
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
    ErrorPose error;
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
    tfg robot(&ur, &tr, &init, &controlPosition, &tool, &fulcrumEstimation, 
          &ftSensor, &tf_listener, &error, prefix);
    robot.initializeRobot(type, p_estimado_init, tool_length, initPosition);
    ros::Duration(1).sleep();
    robot.computeRobotCinematic(tool_length);
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