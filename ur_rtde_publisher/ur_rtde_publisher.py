#!/usr/bin/env python3
from tabnanny import check
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Twist
import numpy as np
import argparse
import logging
import sys
sys.path.append('..')
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import ur_script as ur
import threading
import math
from datetime import datetime

URforce = Twist()
URpose = Twist()

# Parametros
parser = argparse.ArgumentParser()
parser.add_argument('--host', default='192.168.1.50',help='name of host to connect to (localhost)') #169.254.147.101 / 192.168.1.30
parser.add_argument('--port', type=int, default=30004, help='port number (30004)')
parser.add_argument('--samples', type=int, default=0,help='number of samples to record')
parser.add_argument('--frequency', type=int, default=125, help='the sampling frequency in Herz')
parser.add_argument('--config', default='record_configuration.xml', help='data configuration file to use (record_configuration.xml)')
parser.add_argument("--verbose", help="increase output verbosity", action="store_true")
parser.add_argument("--buffered", help="Use buffered receive which doesn't skip data", action="store_true")
parser.add_argument("--binary", help="save the data in binary format", action="store_true")
args = parser.parse_args()

if args.verbose:
    logging.basicConfig(level=logging.INFO)

conf = rtde_config.ConfigFile(args.config)
output_names, output_types = conf.get_recipe('out')

con = rtde.RTDE(args.host, args.port)
con.connect()
   
# Ajustes
con.get_controller_version()
con.send_output_setup(output_names, output_types, frequency=args.frequency)
con.send_start()

# Inicializacion variables
A_TTP = Twist()        # Posicion actual
FORCE = Twist()
POSE = Twist()
q_read = [0,0,0,0,0,0]
force = [0,0,0,0,0,0]

def callback(data):
    global FORCE    
   
    # Se guardan las fuerzas del robot            
    FORCE.linear.x,FORCE.linear.y,FORCE.linear.z,FORCE.angular.x,FORCE.angular.y,FORCE.angular.z = state.actual_TCP_force

def read_daemon():
    global A_TTP
    global q_read
    global force
    global FORCE
    global POSE
    while not rospy.is_shutdown():
        if args.samples > 0 or rospy.is_shutdown():
            keep_running = False
        #rate.sleep()
        try:
            if args.buffered:
                state = con.receive_buffered(args.binary)
            else:
                state = con.receive(args.binary)
            if state is not None:
               
                FORCE.linear.x,FORCE.linear.y,FORCE.linear.z,FORCE.angular.x,FORCE.angular.y,FORCE.angular.z = state.actual_TCP_force
                POSE.linear.x,POSE.linear.y,POSE.linear.z,POSE.angular.x,POSE.angular.y,POSE.angular.z = state.actual_TCP_pose
                speed = state.actual_TCP_speed
                q_read = state.actual_q
                force = state.actual_TCP_force  
                force = state.actual_TCP_force
                rate.sleep()
             
        except rtde.RTDEException:
            break
        except KeyboardInterrupt:
            break
           

def FORCE_variables(pub,rate):

    URforce.linear.x=FORCE.linear.x
    URforce.linear.x = np.around(URforce.linear.x, decimals=4)
    URforce.linear.y=FORCE.linear.y
    URforce.linear.y = np.around(URforce.linear.y, decimals=4)
    URforce.linear.z=FORCE.linear.z
    URforce.linear.z = np.around(URforce.linear.z, decimals=4)
   
    URforce.angular.x=FORCE.angular.x
    URforce.angular.x = np.around(URforce.angular.x, decimals=4)
    URforce.angular.y=FORCE.angular.y
    URforce.angular.y = np.around(URforce.angular.y, decimals=4)
    URforce.angular.z=FORCE.angular.z
    URforce.angular.z = np.around(URforce.angular.z, decimals=4)
     
       
    pub.publish(URforce)
    rate.sleep()

def POSE_variables(pub,rate):

    URpose.linear.x=POSE.linear.x
    URpose.linear.x = np.around(URpose.linear.x, decimals=6)
    URpose.linear.y=POSE.linear.y
    URpose.linear.y = np.around(URpose.linear.y, decimals=6)
    URpose.linear.z=POSE.linear.z
    URpose.linear.z = np.around(URpose.linear.z, decimals=6)
   
    URpose.angular.x=POSE.angular.x
    URpose.angular.x = np.around(URpose.angular.x, decimals=6)
    URpose.angular.y=POSE.angular.y
    URpose.angular.y = np.around(URpose.angular.y, decimals=6)
    URpose.angular.z=POSE.angular.z
    URpose.angular.z = np.around(URpose.angular.z, decimals=6)    
   
       
    pub.publish(URpose)
    rate.sleep()
           

if __name__=='__main__':
    rospy.init_node("ur_pose_rtde_publisher")
    # rospy.Subscriber("ur_hardware_interface/script_command",Twist,callback)
    pubpose = rospy.Publisher('/ur3e/rtde/pose', Twist, queue_size=3)
    pubforce = rospy.Publisher('/ur3e/rtde/force', Twist, queue_size=3)  
    rate=rospy.Rate(125)
   
   
   
    d = threading.Thread(target=read_daemon,name='Daemon')
    d.setDaemon(True)
    d.start()

    print("Startin publinshing force and pose received from UR3e via RTDE in topic /ur3e/rtde")


    while not rospy.is_shutdown():
        try:

            FORCE_variables(pubforce,rate)
            POSE_variables(pubpose,rate)

        except rospy.ROSInterruptException:
            pass
