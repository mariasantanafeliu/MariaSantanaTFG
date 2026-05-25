#include <services/hybridControl.hpp>

hybridControl::hybridControl(ur_script* urScript, UMA_trans* umaTf,
                       Init* initRobot, ErrorPose* composeError,PIDController* composePID, selectTool* selectTool, fulcrum* fp, FTSensor* ftSensor,  tf::TransformListener* tf_listener, std::string t_prefix) : 
                       ur(urScript), tr(umaTf), init(initRobot), controlPosition(composePID), tool(selectTool), fulcrumEstimation(fp), ftSensor(ftSensor), listener(tf_listener){//lista de inicialización
    std::cout << t_prefix << " control build" << std::endl;
    prefix_in = t_prefix;
    // Subscripciones a los topics
    goal_pos_sub_ = nh_.subscribe("/coordinator/goal_position", 1000, &hybridControl::cb_stitchCallback, this);
    //goal_pos_sub_ = nh_.subscribe("/potential_field/cart_vel",1000,&hybridControl::cb_stitchCallback,this);

    fulcrum_sub_ = nh_.subscribe("/coordinator/fulcrum_position", 1000, &hybridControl::cb_fulcrumCallback, this);
    abdomen_effector_force_sub_ = nh_.subscribe("/darel/abdomen_effectorForce_topic", 1000, &hybridControl::cb_abdomenForceCallback, this);
    tissue_force_sub_ = nh_.subscribe("/auto/MyhybridControl/tissue_force", 1000, &hybridControl::cb_tissueForceCallback, this);
    vacuumreposo_sub_ = nh_.subscribe("/bleending/sangre_ok", 1000, &hybridControl::cb_vacuumreposo, this);
    // Publicacion topics
    ttp_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("pose_topic", 1000);
    te_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("effectorFinal_topic", 1000);
    vel_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("velocity_topic", 1000);
    fulcrum_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("fulcrum", 1000);
    force_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("effectorFinal_force", 1000);
    base_force_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("robotbase_force", 1000);
    vacuumposition_pub_= nh_.advertise<std_msgs::Bool>("/posicion_ok", 1000);
    desPose.x = 0.;
    desPose.y = 0.;
    desPose.z = 0.;
    last_desPose.x = 0.;
    last_desPose.y = 0.;
    last_desPose.z = 0.;
    forceAbdomen.resize(3);
    torqueAbdomen.resize(3);
    forceTissue.resize(3);
    torqueTissue.resize(3);
    forceAbdomen[0]= 0;
    forceAbdomen[1]= 0;
    forceAbdomen[2]= 0;
    torqueAbdomen[0]=0;
    torqueAbdomen[1]=0;
    torqueAbdomen[2]=0;
    Fz_filt = 0.0;
    filter_alpha = 0.005;
    error_F = 0.0;
    v_z = 0.0;
    error_integral =0;
    P0[0] = 0.0;
    P0[1] = 0.0;
    P0[2] = 0.0;
    delta_cartesian = Eigen::MatrixXd::Zero(6, 1); // Matriz dinámica 6x1 inicializada con ceros
    weight_base = Eigen::Vector3d(0.0, -0.0, -0.0);//-0.9);
    tool_CoM    = Eigen::Vector3d(0.0, 0.0, 0.12);
    vX = {0.0,0.0,0.0};
    vY = {0.0,0.0,0.0};
    vZ = {0.0,0.0,0.0};
}
hybridControl::~hybridControl(){
    std::cout <<"Leaving gently hybridControl..."<< std::endl;
}
//#######################callback##########################
/*void hybridControl::cb_stitchCallback(const geometry_msgs::Point::ConstPtr& msg){
    desPose.x = msg->x;
    desPose.y = msg->y;
    desPose.z = msg->z;//-0.24
    desPoseReceived = true;
    std::cout <<"cb_stitchCallback..."<< std::endl;
}*/

void hybridControl::cb_stitchCallback(const geometry_msgs::Point::ConstPtr& msg)
{
    std::cout << "cb_stitchCallback..." << std::endl;

    // Primera vez: aceptar directamente
    if(sendWait)
    {
        desPose.x = 0.099;
        desPose.y = 0.442;
        desPose.z = -0.194;
    }else{
        if (!firstPoseReceived)
        {
            desPose = *msg;
            last_desPose = *msg;
            firstPoseReceived = true;
            desPoseReceived = true;
            return;
        }

        // Calcular diferencias
        double dx = std::abs(msg->x - last_desPose.x);
        double dy = std::abs(msg->y - last_desPose.y);
        double dz = std::abs(msg->z - last_desPose.z);

        // Si alguna supera el umbral → actualizar
        if (dx > 0.001 || dy > 0.001 || dz > 0.001)
        {
            desPose = *msg;
            last_desPose = *msg;
            std::cout << "Actualizo desPose" << std::endl;
        }
        else
        {
            // NO actualizas → te quedas con la anterior
            std::cout << "Cambio pequeño, ignoro" << std::endl;
        }
    }

    desPoseReceived = true;
}

void hybridControl::cb_vacuumreposo(const std_msgs::Bool::ConstPtr& msg)
{
    if (!msg->data)
    {
        ROS_INFO("Vacuum reposo activado");
        sendWait = true;
        desPose.x = 0.099;
        desPose.y = 0.442;
        desPose.z = -0.194;
    } else{
        sendWait = false;
    }
}

/*void hybridControl::cb_stitchCallback(const geometry_msgs::Twist::ConstPtr& msg){
    desPose.x = msg->linear.x;
    desPose.y = msg->linear.y;
    desPose.z = msg->linear.z;

    desPoseReceived = true;
}*/

void hybridControl::cb_fulcrumCallback(const geometry_msgs::Point::ConstPtr& msg){
    P0[0] = msg->x;
    P0[1] = msg->y;
    P0[2] = (msg->z - 0.001) - 0.00; //❗
    //std::cout << "fulcrum: " << P0[0] << ", " << P0[1] << ", " << P0[2] << std::endl;
    fulcrumReceived = true;
}

void hybridControl::cb_abdomenForceCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    forceAbdomen[0]=msg->data[0];
    forceAbdomen[1]=msg->data[1];
    forceAbdomen[2]=msg->data[2];
    torqueAbdomen[0]=msg->data[3];
    torqueAbdomen[1]=msg->data[4];
    torqueAbdomen[2]=msg->data[5];
}

void hybridControl::cb_tissueForceCallback(const std_msgs::Float64MultiArray::ConstPtr& msg){
    forceTissue[0]=msg->data[0];
    forceTissue[1]=msg->data[1];
    forceTissue[2]=msg->data[2];
    torqueTissue[0]=msg->data[3];
    torqueTissue[1]=msg->data[4];
    torqueTissue[2]=msg->data[5];
}
//#######################function##########################
Eigen::MatrixXd hybridControl::readTransform(std::string base,std::string tool0){
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
//control de fuerza
std::vector<double> hybridControl::forceControl(const double& Kf, std::vector<double> forces){
    std::vector<double> delta_force(3); 
    delta_force[0] = Kf*forces[0];
    delta_force[1] = Kf*forces[1];
    delta_force[2] = Kf*forces[2];
    return delta_force;
}
Eigen::MatrixXd hybridControl::computeTipForceControl(double Kv, std::vector<double> tipForces_base, const Eigen::MatrixXd& v_cartesian) {
    // Parámetros
    double threshold = 0.6;
    double F_des = 1;
    double F_max_safety = 4;
    double dt = 1.0/125.0;
    
    // 1. Filtrado de fuerza
    auto filterForce = [threshold](double f) {
        return (std::abs(f) > threshold) ? f : 0.0;
    };
    
    double Fz_raw = filterForce(tipForces_base[2]);
    Fz_raw = -1*Fz_raw; //cambio de referencia con tipForces_base
    Fz_filt = filter_alpha * Fz_raw + (1 - filter_alpha) * Fz_filt;
    
    // 2. Error de fuerza
    double error_F = F_des - Fz_filt;
    
    double v_z;
    bool force_control_active = false;
    
    // 3. SEGURIDAD CRÍTICA
    if (Fz_filt > F_max_safety) {
        std::cout << "⚠️ EMERGENCY RETRACT! Force: " << Fz_filt << " N" << std::endl;
        v_z = 0.005;
        error_integral = 0.0;
        force_control_active = true;
    }
    // 4. Control activo cuando hay contacto
    else if (Fz_filt > threshold || error_F < -0.1) {
        
        // 🔑 CLAVE: Permitir comandos de retroceso (hacia arriba)
        if (v_cartesian(2,0) > 0.001) {  // Usuario quiere subir
            std::cout << "📍 User commanded retraction - Following trajectory" << std::endl;
            v_z = v_cartesian(2,0);  // Seguir comando directamente
            error_integral *= 0.8;  // Decaer integral
            force_control_active = false;  // Desactivar control de fuerza
        }
        // Si usuario quiere bajar más, control de fuerza lo limita
        else {
            force_control_active = true;
            
            // Anti-windup
            double max_integral = 0.3;
            
            if (std::abs(error_F) > 0.2 && std::abs(error_F) < 1.0) {
                error_integral += error_F * dt;
                error_integral = std::max(-max_integral, std::min(error_integral, max_integral));
            } else if (std::abs(error_F) < 0.15) {
                error_integral *= 0.9;
            } else {
                error_integral = 0.0;
            }
            
            // Control PI
            double Kp = Kv;
            double Ki = Kv / 10.0;
            
            if (std::abs(error_F) < 0.15) {
                v_z = -Ki * error_integral;
            } else {
                v_z = -Kp * error_F - Ki * error_integral;
            }
            
            // 🔑 MEJORA: Mezclar con comando del usuario si quiere bajar lento
            if (v_cartesian(2,0) < -0.001) {  // Usuario quiere bajar
                // Tomar el mínimo (más conservador)
                double v_z_user = v_cartesian(2,0);
                v_z = std::max(v_z, v_z_user);  // El menos negativo (más lento)
                std::cout << "📍 Blending user command: v_user=" << v_z_user 
                          << " v_force=" << v_z << std::endl;
            }
            
            // Saturación asimétrica
            double v_max_down = 0.02;
            double v_max_up = 0.05;
            
            if (v_z < 0) {
                v_z = std::max(-v_max_down, v_z);
            } else {
                v_z = std::min(v_max_up, v_z);
            }
            
            // Seguridad preventiva
            if (Fz_filt > F_des * 1.3) {
                v_z = std::max(v_z, 0.005);
                std::cout << "⚠️ High force: " << Fz_filt << " N - Retracting" << std::endl;
            }
            
            // Debug
            if (std::abs(error_F) > 0.2) {
                std::cout << "Force Control Active:" << std::endl;
                std::cout << "  Fz_filt: " << Fz_filt << " N" << std::endl; 
                std::cout << "  Fz_raw: " << Fz_raw << " N" << std::endl;
                std::cout << "  Error: " << error_F << " N" << std::endl;
                std::cout << "  Integral: " << error_integral << std::endl;
                std::cout << "  v_z: " << v_z << " m/s (" 
                          << (v_z < 0 ? "DOWN" : "UP") << ")" << std::endl;
            }
        }
    }
    // 5. Modo libre
    else {
        v_z = v_cartesian(2,0);
        error_integral *= 0.8;
        force_control_active = false;
        
        static bool was_in_contact = false;
        if (was_in_contact) {
            std::cout << "✓ Released contact - Free motion mode" << std::endl;
            was_in_contact = false;
        }
        if (Fz_filt > threshold) {
            was_in_contact = true;
        }
    }
    
    // Salida
    Eigen::MatrixXd v_force(4,1);
    v_force << v_cartesian(0,0),
            v_cartesian(1,0),
            v_z,
            0.0;
    return v_force;
}
// funcion that retunr a geometric position
geometry_msgs::Point hybridControl::computeFulcrum(Eigen::MatrixXd E_T_Fp, Eigen::MatrixXd T_E){
    geometry_msgs::Point fulcrum_position;
    Eigen::MatrixXd T(4,4);
    T = T_E * E_T_Fp;
    fulcrum_position.x = T(0,3);
    fulcrum_position.y = T(1,3);
    fulcrum_position.z = T(2,3);
    return fulcrum_position;
}
//round delta_cartesian
void hybridControl::roundDeltaCartesian(Eigen::VectorXd& delta_cartesian) { 
    if (delta_cartesian.size() != 6) {
        std::cerr << "Error: El vector debe tener exactamente 6 elementos." << std::endl;
        return;
    }
    for (int i = 0; i < delta_cartesian.size(); ++i) {
        double rounded = std::round(delta_cartesian[i] * 10000.0) / 10000.0;
        delta_cartesian[i] = (std::abs(rounded) < 1e-5) ? 0.0 : rounded;
    }
}

hybridControl::FTResult hybridControl::compensateForce(const Eigen::Matrix4d& T_E, const Eigen::Vector3d& weight_base, const Eigen::Vector3d& tool_CoM, double res_xy,double res_z, FTSensor* ftSensor){
    FTResult result;
    Eigen::Matrix3d R_E = T_E.topLeftCorner<3,3>();
    std::vector<double> raw;
    ftSensor->readFT(raw);
    Eigen::Vector3d F_tool_raw(raw[0], raw[1], raw[2]);
    Eigen::Vector3d T_tool_raw(raw[3], raw[4], raw[5]);

    // 1️⃣ Deadband: eliminar ruido según resolución del sensor
    F_tool_raw(0) = (std::abs(F_tool_raw(0)) < res_xy) ? 0.0 : F_tool_raw(0);
    F_tool_raw(1) = (std::abs(F_tool_raw(1)) < res_xy) ? 0.0 : F_tool_raw(1);
    F_tool_raw(2) = (std::abs(F_tool_raw(2)) < res_z) ? 0.0 : F_tool_raw(2);
    /*printf("-------------------------------------------****\n");
    printf("raw Fx: %.4f N, base Fy: %.4f N, base Fz: %.4f N\n",
                        F_tool_raw[0], F_tool_raw[1], F_tool_raw[2]);*/
    // 2️⃣ Convertir peso a TOOL
    Eigen::Vector3d weight_tool = R_E.transpose() * weight_base;

    Eigen::Vector3d peso_base(0.0, 0.0, -1.0);  // 1N hacia abajo
    // 3️⃣ Compensar fuerza
    result.F_comp_tool = F_tool_raw - weight_tool;
    // 4️⃣ Torque producido por el peso: τ = r × F
    Eigen::Vector3d torque_weight = tool_CoM.cross(weight_tool);
    result.T_comp_tool = T_tool_raw - torque_weight;

    return result;
}

//funciones
void hybridControl::initializeRobot(int type, double p_estimado_init, double tool_length, std::vector<double> initPosition){
    //p_estimado = p_estimado_init;
    std::cout << "vamos a inicializar: " <<initPosition[0]  << ", " <<initPosition[1]  << ", " <<initPosition[2]  << ", "<<initPosition[3]  << ", "<<initPosition[4]  << ", " << std::endl;
    init->initialize(initPosition,tool0, ur);
    tool->computeTwrist(type, p_estimado_init, tool_length, tr);
    E_T_TTP = tool->E_T_TTP;
    E_T_Fp = tool->E_T_Fp;
    /*std::cout << "type=" << type << std::endl;
    std::cout << "p_estimado_init=" << p_estimado_init << std::endl;
    std::cout << "E_T_TTP=[" << E_T_TTP << "]" << std::endl;
    std::cout << "E_T_Fp=[" << E_T_Fp << "]" << std::endl;*/
    TCP = tool->TCP;//TCP;//DFP
    ur->set_tcp(TCP);
    ros::Duration(0.5).sleep();
}
void hybridControl::computeRobotCinematic(double L){
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
    /*std::cout << "************aspiradora************" << std::endl;
    std::cout << "A_T_E=[" << T_E << "]" << std::endl;
    std::cout << "A_T_TTP=[" << T_TTP << "]" << std::endl;*/
    //forces
    //ftSensor->readFT(forces);
    /*FTResult r = compensateForce(T_E, weight_base, tool_CoM, 0.3, 0.9, ftSensor);
    toolForce<< r.F_comp_tool(0), r.F_comp_tool(1), r.F_comp_tool(2);
    forces= {toolForce(0), toolForce(1), toolForce(2)};
    robotBaseForce = (T_E.topLeftCorner<3,3>()) * toolForce;
    baseForces = {robotBaseForce(0), robotBaseForce(1), robotBaseForce(2)};*/
    /*std::cout << "************fuerzas leidas************" << std::endl;
    std::cout << "forces: "<<forces[0]<<" "<<forces[1]<<" "<<forces[2]<<std::endl;
    std::cout << "baseForces: "<<baseForces[0]<<" "<<baseForces[1]<<" "<<baseForces[2]<<std::endl;*/
    //fulcrum 
    /*if (newFulcrum && fulcrumReceived){
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
        newFulcrum = false;
        E_T_Fp = tr->desp({E_T_Fp(0,3), E_T_Fp(1,3), T_E(2,3) - P0[2]});
    }*/
    if (newFulcrum) fulcrum_position = computeFulcrum(E_T_Fp, T_E);
    newFulcrum = false;
    //Current position from Fulcrum
    /*delta_cartesian[0] = desPose.x - T_TTP(0,3);
    delta_cartesian[1] = desPose.y - T_TTP(1,3);
    delta_cartesian[2] = desPose.z - T_TTP(2,3);*/
    //Des position from Fulcrum
    fulcrum_point_des.x = desPose.x - fulcrum_position.x;
    fulcrum_point_des.y = desPose.y - fulcrum_position.y;    
    fulcrum_point_des.z = desPose.z - fulcrum_position.z;
    //std::cout << "FULCRUM?" << std::endl;
    //
    if (desPoseReceived) {// && (abs(forceAbdomen[0]) < 1) && (abs(forceAbdomen[1]) < 1) && (abs(forceAbdomen[2]) < 1)){ //desPoseReceived
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
        /*std::cout << "+++++++++++++++" << std::endl;
        std::cout << "T_E=[" << T_E << "]"<< std::endl;
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
        velVector[5] = std::atan2(dR(1,0),dR(0,0)); 
        velVector[5]*=1000;
        velVector[5]=round(velVector[5]);
        velVector[5]/=1000;
        //std::cout << "velVector: " << velVector[0] << ", " << velVector[1] << ", " << velVector[2] << ", " << std::endl;
        velLineal << velVector[0]/1 ,velVector[1]/1,velVector[2]/1,0; //AUI VEL IRENE AQUI DE VERDAD AQUI Ñ
        velAngular << velVector[3]/4, velVector[4]/4,0,0;
        
        //std::cout << "+++++++++++++++" << std::endl;
        //if (sendWait) ur->speedl(velVector, 0.5, 0.5);
        /*if (sendWait){
            std::cout << "sendWait:--------------------- "<< std::endl;
            std::cout << "T_dest=[" << T_dest << "]"<< std::endl;
            velVector[0] = 0.099 - T_TTP(0,3);
            velVector[0]*=1000;
            velVector[0]=round( velVector[0]);
            velVector[0]/=1000;
            velVector[0]=velVector[0];
            velVector[1] = 0.442 - T_TTP(1,3);
            velVector[1]*=1000;
            velVector[1]=round( velVector[1]);
            velVector[1]/=1000;
            velVector[1]=velVector[1];
            velVector[2] = -0.194 - T_TTP(2,3);
            velVector[2]*=1000;
            velVector[2]=round( velVector[2]);
            velVector[2]/=1000;
            velVector[2]=velVector[2];
            velVector[3]*=0;
            velVector[4]*=0;
            velVector[5]*=0; 
            ur->speedl(velVector, 0.5, 0.5);
            std::cout << "velVector: " << velVector[0] << ", " << velVector[1] << ", " << velVector[2] << ", " << std::endl;
        } else{
            velVector = {velLineal(0),velLineal(1),velLineal(2),-velAngular(0),velAngular(1),0};
            std::cout << "velVector: " << velVector[0] << ", " << velVector[1] << ", " << velVector[2] << ", " << std::endl;
            ur->speedl(velVector, 0.5, 0.5);
        }*/
        velVector = {velLineal(0),velLineal(1),velLineal(2),0,0,0};//-velAngular(0),velAngular(1),0};
        //std::cout << "velVector: " << velVector[0] << ", " << velVector[1] << ", " << velVector[2] << ", " << velVector[3] << ", " << velVector[4] << ", " << velVector[5] <<std::endl;
        ur->speedl(velVector, 0.5, 0.5);
        
    } else{
        /*std::cout << "fulcrum_point_des=[" << fulcrum_point_des << "]" << std::endl;
        std::cout << "T_E=[" << T_E << "]"<< std::endl;
        std::cout << "T_TTP=[" << T_TTP << "]"<< std::endl;*/
        array_vel.data = {0,0,0,0,0,0};
        vel_pub_.publish(array_vel);
        array_vel.data.clear();
    }
    //Publicar aspiradora
    // ---- Verificación del error absoluto ----
    // Variable que mantiene el estado de si ok_msg.data se ha puesto en true
    bool okMsgPublished = false;

    bool errorSmall = true;

    for (int i = 0; i < 3; ++i)
    {
        // Verificar si alguna de las velocidades es mayor que el umbral
        if (desPoseReceived && std::abs(velVector[i]) >= 0.001) 
        {
            errorSmall = false;
            break;
        }
    }

    // ---- Publicar el booleano en un tópico ----
    std_msgs::Bool ok_msg;

    // Si `okMsgPublished` es falso (es decir, aún no hemos puesto `ok_msg.data` a `true`),
    // y las condiciones son correctas (errorSmall y !sendWait), lo ponemos a true.
    if (!okMsgPublished && errorSmall && !sendWait) {
        ok_msg.data = true;
        okMsgPublished = true;  // Marcar que ya se publicó el `true`
    } else {
        ok_msg.data = false;
    }
    vacuumposition_pub_.publish(ok_msg);
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
    //pub forces
    /*force_msg_.data.clear();
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
    base_force_pub_.publish(base_force_msg_);*/

    //Publish vel
    array_vel.data = velVector;
    vel_pub_.publish(array_vel);
    array_vel.data.clear();
}
 
int main(int argc, char **argv){
    //ros
    ros::init(argc, argv, "MyhybridControl");
    ros::NodeHandle nh_param("~"); // Nota: el uso de '~' para obtener los parámetros relativos al namespace del nodo
    ros::Rate rate(125); //frecuencia a cambiar. De normal a 125 Hz
    //
    ros::Publisher tissue_force_pub_ = nh_param.advertise<std_msgs::Float64MultiArray>("tissue_force", 1000);
    std_msgs::Float64MultiArray tissue_force_msg;
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
    PIDController controlPosition(kp,0,0,0.008,6);
    selectTool tool;
    ErrorPose error;
    fulcrum fulcrumEstimation;
    tf::TransformListener tf_listener;
    FTSensor ftSensor(sensor_ip);
    //----------------
    //urJacobian jacob;
    hybridControl robot(&ur, &tr, &init, &error, &controlPosition, &tool, &fulcrumEstimation, &ftSensor,  &tf_listener,prefix);
    robot.initializeRobot(type, p_estimado_init, tool_length, initPosition);
    ros::Duration(1.5).sleep();
    //cosas del sesnor
    std::vector<double> forces, tissueForces;
    /*if (ftSensor.tareSensor()) {
        std::cout << "Sensor tared successfully\n";
    } else {
        std::cerr << "Failed to tare sensor\n";
    }
    //----tejido
    FTSensor tissueSensor("192.168.1.13");
    if (tissueSensor.tareSensor()) {
        std::cout << "Sensor tared successfully\n";
    } else {
        std::cerr << "Failed to tare sensor\n";
    }*/
    robot.computeRobotCinematic(tool_length);
    std::cout << "vamos al while Hybrid Control" << std::endl;
    while (ros::ok()){
        robot.computeRobotCinematic(tool_length);
        /*if (tissueSensor.readFT(tissueForces)) {
            
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
        tissue_force_pub_.publish(tissue_force_msg);*/
        ros::spinOnce();
        rate.sleep();
    }
}