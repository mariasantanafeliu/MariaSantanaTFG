# include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

#include "geometry_msgs/PoseStamped.h" 
#include "omni_msgs/OmniButtonEvent.h"

//
#include <std_msgs/UInt16.h>

class OMNI_teleop
{
	public:
		OMNI_teleop();
		~OMNI_teleop();
		
		//Topics
		void cb_readButton(const omni_msgs::OmniButtonEvent::ConstPtr& msg);
		void cb_readPose(const geometry_msgs::PoseStamped::ConstPtr& msg);
		//ros::Publisher publisher = node_handle.advertise<omni_msgs::OmniFeedback>("/phantom/force_feedback", 1);
		
		ros::NodeHandle n;
		ros::Subscriber subButton;
		ros::Subscriber subPose;

		int button_grey = 0;
		int white_button = 0;
		//geometry_msgs::PoseStamped pose_msg;
		std::vector<double> phantomPose = {0., 0., 0.};//, 0., 0., 0., 0.}; //pX, pY, pZ, ¿rX, rY, rZ, q?
		std::vector<double> PoseRef = {0., 0., 0.};
		std::vector<double> OrientationRef = {0., 0.,0., 0.};
		Eigen::Quaternion<double> phantomOrientation;
};