#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include "force_position_control_dependecies/ftResponse.h"

class Stitches_Position
{
	public:
		Stitches_Position();//(std::string sensor_id); //ros::NodeHandle n
		~Stitches_Position();
		
		//Topics
		void cb_readStitchesPosition(const geometry_msgs::Point& msg);
		//Variables
		geometry_msgs::Point Position;
		
	private:
		ros::NodeHandle n;
		ros::Subscriber subPosition;
};
