//Clase para el nodo del sensor de fuerza
#include "dependecies/omni_teleop.hpp"

OMNI_teleop::OMNI_teleop() //Constructor
{
	ROS_INFO_STREAM("---OMNI_teleop----");
	subButton= n.subscribe("/phantom/button", 1,  &OMNI_teleop::cb_readButton, this);
	subPose= n.subscribe("/phantom/pose", 1,  &OMNI_teleop::cb_readPose, this);
}

OMNI_teleop::~OMNI_teleop()
{
    ROS_INFO_STREAM("Leaving gently...");
}

// ----------------------------------------------------------------------------------------------------- //
// -------------------------------- TOPICS CALLBACK ------------------------------------- //
// ----------------------------------------------------------------------------------------------------- //

void OMNI_teleop::cb_readButton(const omni_msgs::OmniButtonEvent::ConstPtr& msg)
{
	//ROS_INFO_STREAM("BOTON");
	button_grey = msg->grey_button;
	white_button = msg->white_button;
}

void OMNI_teleop::cb_readPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if (white_button == 1) //mientras el boton gris esté presionado
	{
		phantomPose[0] = msg->pose.position.x - PoseRef[0];
		phantomPose[1] = msg->pose.position.y - PoseRef[1];
		phantomPose[2] = msg->pose.position.z - PoseRef[2];

		phantomOrientation.x()=msg->pose.orientation.x - OrientationRef[0];
		phantomOrientation.y()=msg->pose.orientation.y - OrientationRef[1];
		phantomOrientation.x()=msg->pose.orientation.z - OrientationRef[2];
		phantomOrientation.w()=msg->pose.orientation.w - OrientationRef[3];
	}
	else
	{
		PoseRef = {msg->pose.position.x,msg->pose.position.y,msg->pose.position.z};
		OrientationRef={msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w};
		phantomPose[0] = 0;
		phantomPose[1] = 0;
		phantomPose[2] = 0;
		phantomOrientation.x()=0;
		phantomOrientation.y()=0;
		phantomOrientation.x()=0;
		phantomOrientation.w()=0;
	}
	/*
	phantomPose[0] = msg->pose.position.x;
	phantomPose[1] = msg->pose.position.y;
	phantomPose[2] = msg->pose.position.z;

	phantomOrientation.x()=msg->pose.orientation.x;
    phantomOrientation.y()=msg->pose.orientation.y;
    phantomOrientation.x()=msg->pose.orientation.z;
    phantomOrientation.w()=msg->pose.orientation.w;
		*/
	/*phantomOrientation.setX(msg->pose.orientation.x);
    phantomOrientation.setY(msg->pose.orientation.y);
    phantomOrientation.setZ(msg->pose.orientation.z);
    phantomOrientation.setW(msg->pose.orientation.w);*/
}

/*void OMNI_teleop::cb_readPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	if (white_button == 1) //mientras el boton gris esté presionado
	{
		phantomPose[0] = msg->pose.position.x;
		phantomPose[1] = msg->pose.position.y;
		phantomPose[2] = msg->pose.position.z;
	}
	else
	{
		phantomPose[0] = 0;
		phantomPose[1] = 0;
		phantomPose[2] = 0;
	}

	/*phantomPose[0] = msg->pose.position.x;
	phantomPose[1] = msg->pose.position.y;
	phantomPose[2] = msg->pose.position.z;
}*/
