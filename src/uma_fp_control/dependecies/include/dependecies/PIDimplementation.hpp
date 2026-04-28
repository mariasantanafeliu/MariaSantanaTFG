#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>

class PIDimplementation
{
	public:
		PIDimplementation();
		~PIDimplementation();
		
		//funciones
		void computeVel(std::vector<double> Diff, std::vector<double> CumDiff, std::vector<double> Kp, std::vector<double> Ki, std::vector<double> Kd,std::vector<double> lastError,double elapsed_time);
		
		//HEX variables
		//PID
		double cumError = 0;
		double rateError = 0;
        std::vector<double> ret = {0.0,0.0}; //vel,CumDiff
		std::vector<double> velVector = {0.0,0.0,0.0,0.0,0.0,0.0}; //vel,CumDiff
        typedef struct {double vel; double cummDiff; double last;} MyStruct;
        MyStruct PID[6];
};