//Coordinador 2 robots moviendose al mismo punto que le manda la camara. 
//En este código de experimentacion las coordenadas son las mismas para cada robot, algo que está mal porque no respeta el sistema de coordenadas global
//pero como tampoco uso la camara de verdad, solamente me invento puntos, no pasa nada
//----------INPUTS-------------//
//  haptics
//  camera
//  ontology
//  nº robot
//  robot type
//  robot pose (e.g. ontology will use)
//----------OUTPUTS------------//
//  des position or des displacement
//  tool action (close, open, ...)
//  initial pose
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <string.h>
#include "std_msgs/UInt8MultiArray.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include <cmath>
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include <Eigen/Dense>
#include "omni_msgs/OmniButtonEvent.h"
#include "geometry_msgs/PoseStamped.h" 
#include "std_msgs/Int8.h"

class coordinator {
public:
  coordinator(const std::vector<std::string>& robot_names){
    // Subscripciones a los topics
    //haptic_sub_ = nh_.subscribe("/haptic_topic", 1000, &coordinator::cb_readPose, this);
    haptic_sub_ = nh_.subscribe("/AZUL/phantom/pose", 1000, &coordinator::cb_readPose, this); //"/AZUL/phantom/pose"
    hapticBt_sub_ = nh_.subscribe("/AZUL/phantom/button/white", 1000, &coordinator::cb_readButton, this); //"/AZUL/phantom/button/white"
    //camera_sub_ = nh_.subscribe("/haptic_topic", 1000, &coordinator::cmd_camera, this);
    haptic_sub_rojo_ = nh_.subscribe("/ROJO/phantom/pose", 1000, &coordinator::cb_readPose_rojo, this); //
    hapticBt_sub_rojo_ = nh_.subscribe("/ROJO/phantom/button/white", 1000, &coordinator::cb_readButton_rojo, this); //
    pose_r1_sub_ = nh_.subscribe(robot_names[1] + "/pose_topic", 1000, &coordinator::cmd_pose1, this);
    pose_r2_sub_ = nh_.subscribe(robot_names[0] + "/pose_topic", 1000, &coordinator::cmd_pose2, this);
    //pub
    aspirator_status_pub_ = nh_.advertise<std_msgs::Bool>("/aspirator/status", 1, true); // latch = true

    for (const auto& robot_name : robot_names) {    
      goal_pubs_.push_back(nh_.advertise<geometry_msgs::Point>(robot_name + "/coordinator/goal_position", 1000));
    }

    stitch_.x =0.4871;
    stitch_.y = 0.13958;
    stitch_.z = -0.16655;

    // Inicialización de vectores
    white_button_.resize(2, 0);            // para 2 botones: azul y rojo
    phantomPoseRef_.resize(3, 0.0);        // x, y, z
    phantomPoseRefRojo_.resize(3, 0.0);    // x, y, z
  }
  /*void composeHapticDesPose(geometry_msgs::Point msg) {
    ROS_INFO_STREAM("-----entro cmd_haptic----");
    Tdesp_ << 1, 0, 0, msg.x,
              0, 1, 0, msg.y,
              0, 0, 1, msg.z,
              0, 0, 0, 1;

    dest_ = Tdesp_ * T1_;
    std::cout << "T1_= " << T1_ << std::endl;
    std::cout << "dest_= " << dest_ << std::endl;
    if (msg.z > 10){
      stitch_.x =  stitch_.x;
      stitch_.y = stitch_.y;
      stitch_.z = stitch_.z + 0.03;
    }

    goal_pubs_[1].publish(stitch_);
  }*/
  //callbacks
  void cb_readButton(const std_msgs::Int8::ConstPtr& msg){
    white_button_[0] = msg->data;
  }
  void cb_readButton_rojo(const std_msgs::Int8::ConstPtr& msg){
    white_button_[1] = msg->data;
  }
  void cb_readPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    //ROS_INFO_STREAM("-----entro cb_readPose----");
    if (white_button_[0] == 1){
      phantomPose_.x = msg->pose.position.x - phantomPoseRef_[0];
      phantomPose_.y = msg->pose.position.y - phantomPoseRef_[1];
      phantomPose_.z = msg->pose.position.z - phantomPoseRef_[2];
      //composeHapticDesPose(phantomPose_);
      goal_pubs_[0].publish(phantomPose_);
    }
    else{
      phantomPoseRef_ = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
      phantomPose_.x = 0;
      phantomPose_.y = 0;
      phantomPose_.z = 0;
      goal_pubs_[0].publish(phantomPose_);
    }
  }
  void cb_readPose_rojo(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
    //ROS_INFO_STREAM("-----entro cb_readPose ROJO----");
    if (white_button_[1] == 1){
      phantomPoseRojo_.x = msg->pose.position.x - phantomPoseRefRojo_[0];
      phantomPoseRojo_.y = msg->pose.position.y - phantomPoseRefRojo_[1];
      phantomPoseRojo_.z = msg->pose.position.z - phantomPoseRefRojo_[2];
      //composeHapticDesPose(phantomPose_);
      goal_pubs_[1].publish(phantomPoseRojo_);
      //ROS_INFO_STREAM("-----entro cb_readPose----");
    }
    else{
      phantomPoseRefRojo_ = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
      phantomPoseRojo_.x = 0;
      phantomPoseRojo_.y = 0;
      phantomPoseRojo_.z = 0;
      goal_pubs_[1].publish(phantomPoseRojo_);
    }
  }
  
  //other read pose but now the data type is geometry_msgs::Point
  /*void cb_readPose(const geometry_msgs::Point::ConstPtr& msg){
    ROS_INFO_STREAM("-----entro cmd_haptic----");
    phantomPose_.x = msg->x;
    phantomPose_.y = msg->y;
    phantomPose_.z = msg->z;
    composeHapticDesPose(phantomPose_);
  }*/
  //void cmd_camera(const std_msgs::UInt8MultiArray::ConstPtr& msg){
  /*void cmd_camera(const geometry_msgs::Point::ConstPtr& msg){
    ROS_INFO_STREAM("-----entro cmd_camera----");

    phantomPose_.x = msg->x;
    phantomPose_.y = msg->y;
    phantomPose_.z = msg->z;
    composeHapticDesPose(phantomPose_);
  }*/

  void cmd_pose1(const std_msgs::Float64MultiArray::ConstPtr& msg){
    //ROS_INFO_STREAM("-----entro cmd_pose1----");
    // Crear una matriz Eigen::MatrixXd para almacenar los datos del mensaje
    Eigen::MatrixXd matrix(4, 4);
    // Copiar los datos del mensaje a la matriz Eigen
    int index = 0;
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            matrix(i,j) = msg->data[index++];
        }
    }
    T1_ = matrix;
    // === CHUPON

    // Calcular la distancia euclidiana al punto stitch_
    /*double dx = T1_(0, 3) - stitch_.x;
    double dy = T1_(1, 3) - stitch_.y;
    double dz = T1_(2, 3) -(-0.16655);
    double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

    // Comparar distancia y publicar estado del aspirador
    std_msgs::Bool msg_out;
    if (distance < 0.01) {
        msg_out.data = true;
    } else {
        msg_out.data = false;
    }
    aspirator_status_pub_.publish(msg_out);*/
  }
  void cmd_pose2(const std_msgs::Float64MultiArray::ConstPtr& msg){
    //ROS_INFO_STREAM("-----entro cmd_pose2----");
    // Crear una matriz Eigen::MatrixXd para almacenar los datos del mensaje
    Eigen::MatrixXd matrix(4, 4);
    // Copiar los datos del mensaje a la matriz Eigen
    int index = 0;
    for (int j = 0; j < 4; ++j) {
        for (int i = 0; i < 4; ++i) {
            matrix(i,j) = msg->data[index++];
        }
    }
    T2_ = matrix;
  }
private:
  ros::NodeHandle nh_;
  //sub
  ros::Subscriber haptic_sub_;
  ros::Subscriber hapticBt_sub_;
  ros::Subscriber haptic_sub_rojo_;
  ros::Subscriber hapticBt_sub_rojo_;
  ros::Subscriber camera_sub_;
  ros::Subscriber ontology_sub_;
  ros::Subscriber num_sub_;
  ros::Subscriber type_sub_;
  ros::Subscriber pose_r1_sub_;
  ros::Subscriber pose_r2_sub_;
  //pub
  ros::Publisher aspirator_status_pub_;
  std::vector<ros::Publisher> goal_pubs_;
  //variables
  geometry_msgs::Point stitch_;
  geometry_msgs::Point phantomPose_;
  std::vector<double> phantomPoseRef_;
  geometry_msgs::Point phantomPoseRojo_;
  std::vector<double>  phantomPoseRefRojo_;
  Eigen::MatrixXd T1_;
  Eigen::MatrixXd T2_;
  Eigen::MatrixXd dest_;
  Eigen::Matrix4d Tdesp_ = Eigen::Matrix4d::Zero();
  int grey_button_;
  std::vector<int> white_button_;
};

int main(int argc, char** argv) {
  // Inicializa el nodo ROS
  ros::init(argc, argv, "coordinator_node");
  // Crea un manejador de nodo ROS
  ros::NodeHandle nh;
  // Lee los parámetros de ROS
  std::string robot_names_str;
  nh.param<std::string>("robot_names", robot_names_str, "alice,bob");
  // Elimina los espacios en blanco de la cadena de nombres de los robots
  robot_names_str.erase(std::remove_if(robot_names_str.begin(), robot_names_str.end(), ::isspace), robot_names_str.end());
  // Extrae los nombres de los robots del string
  std::vector<std::string> robot_names;
  std::stringstream ss(robot_names_str);
  std::string token;
  while (std::getline(ss, token, ',')) {
    robot_names.push_back(token);
  }
  // Crea una instancia de la clase coordinator con los parámetros obtenidos
  coordinator coord(robot_names);
  // Gira el bucle ROS
  ros::spin();
  return 0;
}