// lee por teclados comandos de teleoperacion
// el resultado se mando por el topic cmd_key
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h>
#include <string.h>
#include "geometry_msgs/Point.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard");
    ros::NodeHandle n;
    //Para realizar la lectura por pantalla
    system("stty raw"); //evita presionar enter tras cada letra
    bool salir=false; //sale tras pulsar x
    ros::Publisher pub = n.advertise<geometry_msgs::Point>("haptic_topic", 1000);
    while (ros::ok() && !salir){
        char input=getchar();
        geometry_msgs::Point msg;
        msg.x = 0;
        msg.y = 0;
        msg.z = 0;
        double paso_fino = 0.001;
        double paso_medio = 0.005;

        bool tecla_valida = false;

        switch (input){
            case '1': 
                msg.x = paso_medio;
                ROS_INFO("\r\n X + 5mm \r\n");
                tecla_valida = true;
                break;  
            case '2':
                msg.y = paso_medio;
                ROS_INFO("\r\n Y + 5mm \r\n");
                tecla_valida = true;
                break; 
            case '3': 
                msg.z = paso_medio;
                ROS_INFO("\r\n Z + 5mm \r\n");
                tecla_valida = true;
                break;    
            case '4': 
                msg.x = -paso_medio;
                ROS_INFO("\r\n X - 5mm \r\n");
                tecla_valida = true;
                break;  
            case '5':
                msg.y = -paso_medio;
                ROS_INFO("\r\n Y - 5mm \r\n");
                tecla_valida = true;
                break; 
            case '6': 
                msg.z = -paso_medio;
                ROS_INFO("\r\n Z - 5mm \r\n");
                tecla_valida = true;
                break;  
            case '7':
                msg.x = -0.0318;
                msg.y = -0.0318;
                msg.z = -0.01;
                ROS_INFO("\r\n Diagonal X Y \r\n");
                tecla_valida = true;
                break;  
            case '8':
                msg.x = -0.0283;
                msg.y = -0.0283;
                msg.z = -0.02;
                ROS_INFO("\r\n Diagonal -X -Y -Z \r\n");
                tecla_valida = true;
                break;
            case '9':
                msg.x = 0.0283;
                msg.y = 0.0283;
                msg.z = 0.02;
                ROS_INFO("\r\n Diagonal -X -Y . Pulsa x para salir\r\n");
                tecla_valida = true;
                break;
            case 'o':
                msg.x = -0.0354;
                msg.y = -0.0354;
                msg.z = -0.02;
                ROS_INFO("\r\n Diagonal -X -Y -Z \r\n");
                tecla_valida = true;
                break;
            case 'p':
                msg.x = 0.0354;
                msg.y = 0.0354;
                msg.z = 0.02;
                ROS_INFO("\r\n Diagonal -X -Y . Pulsa x para salir\r\n");
                tecla_valida = true;
                break;
            case 'b':
                msg.x = 0.02; 
                ROS_INFO("\r\n Salto en X y espera \r\n");
                tecla_valida = true;
                break;
            case 'v':
                msg.x = -0.04; 
                ROS_INFO("\r\n Salto en -X y espera \r\n");
                tecla_valida = true;
                break;
            case 'c':{
                /*double T = 10.0;
                ros::Time start_time = ros::Time::now();

                ROS_INFO("\r\n Movimiento Chirp en X \r\n");
                msg.x = -0.025; 
                pub.publish(msg);
                ros::Duration(1.1).sleep();
                while (ros::ok() && (ros::Time::now() - start_time).toSec() < T){
                    double t = (ros::Time::now() - start_time).toSec();

                    double freq = 0.1 + 0.8 * (t/T);

                    if (sin(2* M_PI * freq * t) >= 0){
                        msg.x = 0.05; //para fase1 2cm
                    } else {
                        msg.x = -0.05; //para fase1 2cm
                    }
                    ROS_INFO("\r\n moviendo \r\n");
                    pub.publish(msg);
                    ros::Rate(20).sleep();
                }
                ROS_INFO("\r\n Chirp terminado \r\n");
                msg.x = 0;
                break;*/

                double T = 10.0;
                ros::Time start_time = ros::Time::now();

                ROS_INFO("\r\n Movimiento Chirp en X \r\n");

                msg.x = -0.02;
                //msg.y = 0.02;
                pub.publish(msg);
                ros::Duration(1.1).sleep();

                while (ros::ok() && (ros::Time::now() - start_time).toSec() < T){
                    double t = (ros::Time::now() - start_time).toSec();

                    double f0 = 0.1;
                    double k = (0.8 - f0)/T;
                    double fase = 2* M_PI * (f0 * t + 0.5 * k * t * T);

                    double amp_x = 0.04;
                    //double amp_y = 0.04;
                    double current_x = 0.03 * sin(fase);

                    msg.x = -amp_x * sin(fase);
                    //msg.y = amp_y * sin(fase);
                    ROS_INFO("\r\n moviendo \r\n");
                    pub.publish(msg);
                    ros::Rate(20).sleep();
                }

                msg.x = 0.01;
                pub.publish(msg);
                ROS_INFO("\r\n Chirp terminado \r\n");
                tecla_valida = false;
                break;

            }
            case 'd':{
                msg.x = 0.0075;
                pub.publish(msg);
                ros::Duration(1.8).sleep();

                int total_pasos = 40;
                double radio = 0.015; //para fase1 2cm
                double paso_recto = (2.0 * M_PI * radio)/ total_pasos;
                int i;

                ROS_INFO("\r\n Trayectoria circular XY \r\n");
                for (i = 0; i < total_pasos; i++){
                    double angulo = (2.0 * M_PI * i)/ total_pasos;
                    msg.x = -paso_recto * sin(angulo);
                    msg.y = paso_recto * cos(angulo);
                    msg.z = -0.0008;//añadido experimento 8
                    pub.publish(msg);
                    ros::Duration(0.6).sleep();
                }
                msg.x = -0.0075;
                msg.y = 0;
                pub.publish(msg);
                tecla_valida = false;
                break;
            }
            case 'x':
                salir = true;
                system("stty cooked");
                tecla_valida = false;
                break;    
                
            default:
                tecla_valida = false;
                break;
        }
        if (tecla_valida){
            pub.publish(msg); //escucha por si alguien quiere hablar con mi nodo 
        }
    }
  return 0;
}