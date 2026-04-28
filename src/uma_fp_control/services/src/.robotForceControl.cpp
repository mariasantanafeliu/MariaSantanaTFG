#include <services/robotForceControl.hpp>

ur3ForceControl::ur3ForceControl(ur_script* urScript, UMA_trans* umaTf, PoseDesJM* estimatePose,
                       Init* initRobot, ErrorPose* composeError, forceControl* composeforceControl, 
                       PIDController* composePID, PIDController* composePID2, Cinematic* robotTransf,
                       selectTool* selectTool, fulcrum* fp,std::string t_prefix) : 
                       ur(urScript), tr(umaTf), Pdest(estimatePose),
                       init(initRobot), error(composeError), Kforce(composeforceControl), 
                       controlPosition(composePID), controlForce(composePID2), T(robotTransf),
                       tool(selectTool), fulcrumEstimation(fp){//lista de inicialización
    std::cout << t_prefix << " control build" << std::endl;
    prefix_in = t_prefix;
    // Subscripciones a los topics
    goal_pos_sub_ = nh_.subscribe("/"+t_prefix+"/coordinator/goal_position", 1000, &ur3ForceControl::cb_stitchCallback, this);
    force_sensor_sub_ = nh_.subscribe("/ur3e/rtde/force", 1000, &ur3ForceControl::cb_readForceTorque, this);//"/HEX_70_XE_1/" //"/fulcrum_force_topic" //"/ur3e/rtde/force"
    surface_sub_ = nh_.subscribe("/sensor", 1000, &ur3ForceControl::cb_readSurface, this);
    // Publicacion topics
    ttp_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("pose_topic", 1000);
    vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("velocity_topic", 1000);
    te_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("effectorFinal_topic", 1000);
    error_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("error_topic", 1000);
    reset_sensor_pub_ = nh_.advertise<std_msgs::String>("/"+t_prefix+"/ur_hardware_interface/script_command",1000);
    last_desPose.x = 0;
    last_desPose.y = 0;
    last_desPose.z = 0;
    desPose.x = 0.;
    desPose.y = 0.;
    desPose.z = 0.;
    desPoseCom.x = 0.0;
    desPoseCom.y = 0.0;
    desPoseCom.z = 0.0;
}
ur3ForceControl::~ur3ForceControl(){
    std::cout <<"Leaving gently ur control..."<< std::endl;
}
//#######################callback##########################
void ur3ForceControl::cb_stitchCallback(const geometry_msgs::Point::ConstPtr& msg){
    desPose.x = msg->x;
    desPose.y = msg->y;
    desPose.z = msg->z;
    move = true;
    //td::cout << "des pose= " << desPose.x << " ,"<< desPose.y << " ,"<< desPose.z << std::endl;
}
/*void ur3ForceControl::cb_readForceTorque(const uma_fp_control::ftResponse& msg){
	HEXForce[0]=(msg.force.x);
	HEXForce[1]=(msg.force.y);
	HEXForce[2]=(msg.force.z);
	HEXtorque[0]=(msg.torque.x);
	HEXtorque[1]=(msg.torque.y);
	HEXtorque[2]=(msg.torque.z);
}*/
void ur3ForceControl::cb_readForceTorque(const geometry_msgs::Twist::ConstPtr& msg){
    //reset sensor
    if(first_force_reading){
        std_msgs::String reset;
        reset.data = "zero_ftsensor()";
        reset_sensor_pub_.publish(reset);
        first_force_reading = false;
    }
    // fuerza base robot
    forceRobotBase << msg->linear.x, msg->linear.y, 0;//msg->linear.z;
    torqueRobotBase << msg->angular.x, msg->angular.y, 0;//msg->angular.z;
    // fuerza efector final robot
    Eigen::Matrix3d R = T_TTP.topLeftCorner<3,3>(); 
    forceRobotTTP = R.inverse()*forceRobotBase;
    torqueRobotTTP = R.inverse()*torqueRobotBase; 
    //std::cout <<" force->" <<msg->linear.x<<" "<<msg->linear.y<<" "<< msg->linear.z<< std::endl;
}
/*void ur3ForceControl::cb_readForceTorque(const geometry_msgs::Twist::ConstPtr& msg){
    forceRobotBase << msg->linear.x, msg->linear.y, msg->linear.z;
    torqueRobotBase << msg->angular.x, msg->angular.y, msg->angular.z;
    // fuerza efector final robot
    Eigen::Matrix3d R = T_TTP.topLeftCorner<3,3>();
    forceRobotTTP = R.inverse()*forceRobotBase;
    torqueRobotTTP = R.inverse()*torqueRobotBase;
}*/
void ur3ForceControl::cb_readSurface(const geometry_msgs::Point::ConstPtr& msg){
}
//#######################fuzzy system##########################
FuzzySystem* createFisRHO() {
    auto* system = new FuzzySystem();
    std::cout << "Creating FuzzySystem..." << std::endl;
    // Crear variables lingüísticas
    auto* forceDirection = new LinguisticVariable("ForceDirection", -1, 1);
    forceDirection->addMF(new TrapezoidalMF("n_F", -1.11651, -1.11651, -0.2, -0.1));
    forceDirection->addMF(new TrapezoidalMF("p_F", 0.1, 0.2, 1.8, 2));

    auto* despDirection = new LinguisticVariable("despDirection", -1, 1);
    despDirection->addMF(new TrapezoidalMF("n_d", -1.11651, -1.11651, -0.2, -0.1));
    despDirection->addMF(new TrapezoidalMF("p_d", 0.1, 0.2, 1.8, 2));

    auto* magnitudeForce = new LinguisticVariable("magnitudeForce", 0, 5);
    magnitudeForce->addMF(new TriangularMF("super_low", 0.4, 0.4, 1.1667));
    magnitudeForce->addMF(new TriangularMF("low", 0.7833, 1.55, 2.3167));
    magnitudeForce->addMF(new TriangularMF("medium", 1.9333, 2.7, 3.4667));
    magnitudeForce->addMF(new TriangularMF("hight", 3.0833 ,3.85, 4.6167));
    magnitudeForce->addMF(new TriangularMF("super_hight", 4.2333, 5, 5));
    magnitudeForce->addMF(new TriangularMF("null", 0, 0, 0.4));

    auto* incremento = new LinguisticVariable("Incremento", -0.06, 0.06);
    incremento->addMF(new TriangularMF("sH_nRho", -0.06, -0.054, -0.0437351916376307));
    incremento->addMF(new TriangularMF("M_nRho", -0.0439024390243902, -0.0353728222996516, -0.0245017421602787));
    incremento->addMF(new TriangularMF("L_nRho", -0.0326968641114983, -0.0266968641114983, -0.0102857142857143));
    incremento->addMF(new TriangularMF("L_pRho", 0.0129616724738676, 0.0223484320557491, 0.0353728222996516));
    incremento->addMF(new TriangularMF("M_pRho", 0.0214912891986063, 0.0298536585365854, 0.044404181184669));
    incremento->addMF(new TriangularMF("sH_pRho", 0.0442369337979094, 0.054, 0.06));
    incremento->addMF(new TriangularMF("sL_nRho", -0.0238328, -0.0113177, -0.006));
    incremento->addMF(new TriangularMF("H_nRho", -0.0534564459930314, -0.0452404181184669, -0.0345365853658537));
    incremento->addMF(new TriangularMF("sL_pRho", 0.006, 0.00878049, 0.0255052));
    incremento->addMF(new TriangularMF("H_pRho", 0.0250871080139373, 0.0412264808362369, 0.0520975609756098));
    incremento->addMF(new TriangularMF("null", -0.005, 0, 0.005));

    system->addInput(forceDirection);
    system->addInput(despDirection);
    system->addInput(magnitudeForce);
    system->setOutput(incremento);

    // Añadir reglas
    system->addRule(FuzzyRule({0, 0, 0}, 6, 1.0));  // 1 1 1, 7 (1)
    system->addRule(FuzzyRule({0, 0, 1}, 2, 1.0));  // 1 1 2, 3 (1)
    system->addRule(FuzzyRule({0, 0, 2}, 1, 1.0));  // 1 1 3, 2 (1)
    system->addRule(FuzzyRule({0, 0, 3}, 7, 1.0));  // 1 1 4, 8 (1)
    system->addRule(FuzzyRule({0, 0, 4}, 0, 1.0));  // 1 1 5, 1 (1)
    system->addRule(FuzzyRule({0, 1, 0}, 8, 1.0));  // 1 2 1, 9 (1)
    system->addRule(FuzzyRule({0, 1, 1}, 3, 1.0));  // 1 2 2, 4 (1)
    system->addRule(FuzzyRule({0, 1, 2}, 4, 1.0));  // 1 2 3, 5 (1)
    system->addRule(FuzzyRule({0, 1, 3}, 9, 1.0));  // 1 2 4, 10 (1)
    system->addRule(FuzzyRule({0, 1, 4}, 5, 1.0));  // 1 2 5, 6 (1)
    system->addRule(FuzzyRule({1, 0, 0}, 8, 1.0));  // 2 1 1, 9 (1)
    system->addRule(FuzzyRule({1, 0, 1}, 3, 1.0));  // 2 1 2, 4 (1)
    system->addRule(FuzzyRule({1, 0, 2}, 4, 1.0));  // 2 1 3, 5 (1)
    system->addRule(FuzzyRule({1, 0, 3}, 9, 1.0));  // 2 1 4, 10 (1)
    system->addRule(FuzzyRule({1, 0, 4}, 5, 1.0));  // 2 1 5, 6 (1)
    system->addRule(FuzzyRule({1, 1, 0}, 6, 1.0));  // 2 2 1, 7 (1)
    system->addRule(FuzzyRule({1, 1, 1}, 2, 1.0));  // 2 2 2, 3 (1)
    system->addRule(FuzzyRule({1, 1, 2}, 1, 1.0));  // 2 2 3, 2 (1)
    system->addRule(FuzzyRule({1, 1, 3}, 7, 1.0));  // 2 2 4, 8 (1)
    system->addRule(FuzzyRule({1, 1, 4}, 0, 1.0));  // 2 2 5, 1 (1)
    //In matlab I can put null, here I have to contemplate the 4 possibilities
    system->addRule(FuzzyRule({0, 0, 5}, 10, 1.0));  // 0 0 6, 11 (1) : 1
    system->addRule(FuzzyRule({1, 1, 5}, 10, 1.0));  // 0 0 6, 11 (1) : 1
    system->addRule(FuzzyRule({0, 1, 5}, 10, 1.0));  // 0 0 6, 11 (1) : 1
    system->addRule(FuzzyRule({1, 0, 5}, 10, 1.0));  // 0 0 6, 11 (1) : 1
    std::cout << "FuzzySystem created." << std::endl;
    return system;
}
//#######################function##########################
Eigen::MatrixXd ur3ForceControl::readTransform(std::string base,std::string tool0){
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
//funciones
void ur3ForceControl::computeFulcrum(FuzzySystem* fuzzySystem, double p_anterior){
    /*double forceMagnitude = forceRobotTTP.norm();
    double torqueMagnitude = torqueRobotTTP.norm();
    if (forceMagnitude > 1.2) {
        p_estimado = torqueMagnitude / forceMagnitude;
        if (p_estimado>TCP[2]){
            p_estimado = p_anterior;
        } else{
        }
    } else{
        p_estimado = p_anterior;
    }*/
    //incremento X
    double despDirectionX = ((desPoseCom.x - last_desPose.x) > 0) ? 1 : -1; 
    double forceDirectionX = (forceRobotBase(0) > 0) ? 1 : -1;
    //std::cout << "fuzzySystem X inputs: " << forceDirectionX << " " << despDirectionX << " " << forceRobotBase(0) << std::endl;
    double incrementoX = fuzzySystem->evaluate({forceDirectionX, despDirectionX, forceRobotBase(0)});
    //std::cout << "fuzzySystem X output: " << incrementoX << std::endl;
    //incremento Y
    /*double despDirectionY = ((desPoseCom.y - last_desPose.y) > 0) ? 1 : -1; 
    double forceDirectionY = (forceRobotBase(1) > 0) ? 1 : -1;
    std::cout << "fuzzySystem Y inputs: " << forceDirectionY << " " << despDirectionY << " " << forceRobotBase(1) << std::endl;
    double incrementoY = fuzzySystem->evaluate({forceDirectionY, despDirectionY, forceRobotBase(1)});
    std::cout << "fuzzySystem Y output: " << incrementoY << std::endl;

    std::cout << "p_estimado: " << p_estimado << std::endl;*/
    //p_estimado += incrementoX + incrementoY;
    //p_estimado = std::round(p_estimado * 1000.0) / 1000.0;
}
void ur3ForceControl::initializeRobot(int type, double p_estimado_init, double tool_length, std::vector<double> initPosition){
    p_estimado = p_estimado_init;
    init->initialize(initPosition,tool0, ur);
    tool->computeTwrist(type, p_estimado_init, tool_length, tr);
    E_T_TTP = tool->E_T_TTP;
    E_T_Fp = tool->E_T_Fp;
    TCP = tool->TCP;//DFP//TCP
    ur->set_tcp(TCP);
    ros::Duration(0.5).sleep();
}
void ur3ForceControl::computeForceControl(double Kf){
    std::vector<double> Force = {-forceRobotBase[0], -forceRobotBase[1], -forceRobotBase[2]};
    if ((std::abs(Force[0]) > 0.5) || (std::abs(Force[1]) > 0.5) || (std::abs(Force[2]) > 0.5)){
        deltaXF = controlForce->calculate(Force);
    }
    else{
        deltaXF[0] = 0;
        deltaXF[1] = 0;
        deltaXF[2] = 0;
    }
    deltaXF[0] = 0;
    deltaXF[1] = 0;
    deltaXF[2] = 0;
    desPoseCom.x = desPose.x - deltaXF[0];
    desPoseCom.y = desPose.y - deltaXF[1];
    desPoseCom.z = desPose.z - deltaXF[2];
}
void ur3ForceControl::computeRobotCinematic(FuzzySystem* fuzzySystem,bool flagFp, int type){
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
    //std::cout << "T_TTP: " << T_TTP << std::endl;
    //computeFulcrum(fuzzySystem, p_estimado);
    //T->computeCinematicAlphaBeta(TTP_dest, T_E, T_TTP, desPoseCom, last_desPose, E_T_TTP, E_T_Fp, T_Fp, p_estimado,TCP[2], flagFp, tr);
    T->computeCinematicTf(TTP_dest, T_E, T_TTP, desPoseCom, last_desPose, prefix_in, E_T_TTP, E_T_Fp ,T_Fp, p_estimado,flagFp,fulcrumForces, tr, Pdest);
    //T->computeCinematicIncrement(TTP_dest, T_E, T_TTP, desPoseCom, last_desPose, E_T_TTP, E_T_Fp, T_Fp, p_estimado, TCP[2], flagFp, tr);
    TTP_dest = T->TTP_dest_orientado;
    //std::cout << "TTP_dest: " << TTP_dest << std::endl;
    T_Fp = T->T_Fp;
    /*if (!flagFp && first_time) {
        first_time = false;
        std::vector<double> polar_angles = {0, 1.5708, p_estimado};
        std::cout << "----------------------------" << std::endl;
        TTP_dest = tr->desp({T_Fp(0,3), T_Fp(1,3), T_Fp(2,3)}) * tr->rotZ(polar_angles[0]) * tr->rotY(-3.141592653589793/2-polar_angles[1]) * tr->desp({0,0, TCP[2]-polar_angles[2]});
        std::cout << "TTP_dest"<< TTP_dest<< std::endl;
        //init->initializePolar(polar_angles, T_Fp, TCP, ur, tr);
        std::cout << "----------------------------" << std::endl;
    }*/
    last_desPose = desPoseCom;
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
void ur3ForceControl::computePoseError(){
    diffPose = error->computeErrorTf(TTP_dest, T_TTP);
    for (int i = 0; i < 6; ++i) {
        error_.data.push_back(diffPose[i]);
    }
    error_pub_.publish(error_);
    error_.data.clear();
}
void ur3ForceControl::computeRobotVel(){
    velVector = controlPosition->calculate(diffPose);
    velLineal << velVector[0] ,velVector[1],velVector[2],1;
    velAngular << velVector[3], velVector[4],velVector[5],0;
    //std::cout << "T_TTP: " << T_TTP << std::endl;
    //transformadas
    //velLineal << 0,0,0,0;
    velAngular << 0,0,0,0;
    velAngular = T_TTP * velAngular; //la diferencia de posicion se hace respecto a la base, PERO el de angulo no, por eso esto es necesario POR USAR dR
    velVector = {velLineal(0),velLineal(1),velLineal(2),velAngular(0),velAngular(1),0};
    //velVector = {velLineal(0),velLineal(1),velLineal(2),0,0,0};
    //if(move) std::cout << "velVector" << velVector[0] <<"," << velVector[1] << "," << velVector[2] << "," << velVector[3] << "," << velVector[4] << "," << velVector[5] << std::endl;
    //send velocity to robot
    //std::cout << "velVector" << velVector[0] <<"," << velVector[1] << "," << velVector[2] << "," << velVector[3] << "," << velVector[4] << "," << velVector[5] << std::endl;
    ur->speedl(velVector, 0.1, 0.1);
    array_vel.data = velVector;
    vel_pub_.publish(array_vel);
}   
int main(int argc, char **argv){
    //ros
    ros::init(argc, argv, "MyControlForce");
    ros::NodeHandle nh_param("~"); // Nota: el uso de '~' para obtener los parámetros relativos al namespace del nodo
    ros::Rate rate(125); //frecuencia a cambiar. De normal a 125 Hz
    //launch param
    std::string prefix;
    double kp, ki, kd, kf;
    double p_estimado_init, tool_length, base, shoulder, elbow, wrist1, wrist2, wrist3;
    int type;
    std::vector<double> initPosition;
    nh_param.param<std::string>("prefix", prefix, "darel");
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
    PIDController controlPosition(kp,0,kd,0.008,6);
    PIDController controlForce(kf,ki,0,0.008,3); 
    Cinematic T;
    selectTool tool;
    fulcrum fulcrumEstimation;
    //FuzzySystem fuzzySystem;
    FuzzySystem* fuzzySystem = createFisRHO();
    ur3ForceControl robot(&ur, &tr, &Pdest, &init, &error, &Kforce, &controlPosition, &controlForce, &T, &tool, &fulcrumEstimation, prefix);
    robot.initializeRobot(type, p_estimado_init, tool_length, initPosition);
    ros::Duration(0.5).sleep();
    robot.computeRobotCinematic(fuzzySystem,true,type);
    std::cout << "vamos al while" << std::endl;
    //std::vector<double> velVector = {0.0,0.0,0.0,0.0,0.0,0.0};
    double i = 0.0;
    while (ros::ok()){
        robot.computeForceControl(kf);
        robot.computeRobotCinematic(fuzzySystem,false,type);
        robot.computePoseError();
        robot.computeRobotVel();
        //ur.speedl(velVector, 0.2, 0.2);
        ros::spinOnce();
        rate.sleep();
    }
}