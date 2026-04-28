// Includes

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <omni_msgs/OmniFeedback.h>

#include <iostream>
#include <stdio.h>
#include <unistd.h>

#include "dependecies/hex_ft_udp.hpp"
int main(int argc, char **argv){
    //ros
    ros::init(argc, argv, "feedback");
    ros::NodeHandle nh_param("~");
    ros::Rate rate(1000);

    std::string prefix, sensor_ip;
    nh_param.param<std::string>("sensor_ip", sensor_ip, "192.168.1.1");
    nh_param.param<std::string>("prefix", prefix, "AZUL");

    FTSensor ftSensor(sensor_ip);
    ros::Publisher force_pub = nh_param.advertise<omni_msgs::OmniFeedback>("/"+prefix+ "/phantom/force_feedback", 1);

    std::vector<double> forces;
    if (ftSensor.tareSensor()) {
        std::cout << "Sensor tared successfully\n";
    } else {
        std::cerr << "Failed to tare sensor\n";
    }
    omni_msgs::OmniFeedback feedback_msg;
    feedback_msg.position.x = 0.0;
    feedback_msg.position.y = 0.0;
    feedback_msg.position.z = 0.0;

    std::cout << "vamos al while" << std::endl;
    while (ros::ok()){
        if (ftSensor.readFT(forces)) {
            //printf("Fx: %.4f N, Fy: %.4f N, Fz: %.4f N, Tx: %.4f Nm, Ty: %.4f Nm, Tz: %.4f Nm\n",
            //        forces[0], forces[1], forces[2], forces[3], forces[4], forces[5]);
        } else {
            std::cerr << "Failed to read sensor \n";
        }

        if (std::fabs(forces[0]) > 0.7 || std::fabs(forces[1]) > 0.7 || std::fabs(forces[2]) > 0.7){
            feedback_msg.force.x = forces[0] *1.4;
            feedback_msg.force.y = forces[1] *1.4;
            feedback_msg.force.z = forces[2] *1.4;
        }
        else{
            feedback_msg.force.x = 0;
            feedback_msg.force.y = 0;
            feedback_msg.force.z = 0;
        }
        force_pub.publish(feedback_msg);

        ros::spinOnce();
        rate.sleep();
    }
}