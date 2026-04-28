#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include <string>
#include <eigen_conversions/eigen_msg.h>
//#include <dependecies/fulcrumEstimation.hpp>
#include <dependecies/computePoseDes_JM.hpp>
#include "dependecies/uma_transf.hpp"


class forceControl
{
	public:
		forceControl();
		~forceControl();
		
		//funciones
		void computeForceControl(std::vector<double> sensor, double tool,
                                 double p_anterior, double Fdes, double Kf,
                                 Eigen::MatrixXd T_Fp, Eigen::MatrixXd T_FpOG,
                                 PoseDesJM* Pdest,UMA_trans* tr);

        /*typedef struct {
            double p_anterior; 
            double RPYdest[2];
            Eigen::MatrixXd T_Fp;
            Eigen::MatrixXd T_FpOG;
        } forceControlParameter;
        forceControlParameter forceFulcrum;*/
        typedef struct {
            double p_anterior; 
            Eigen::MatrixXd TTP_dest_orientado;;
            Eigen::MatrixXd T_Fp;
            //Eigen::MatrixXd T_FpOG;
        } forceControlParameter;
        forceControlParameter forceFulcrum;

    private:
        std::vector<double> DespP = {0,0,0};
        std::vector<double> Desp_rho = {0,0,0};
        std::vector<double> Xf={0,0,0,};
        double p_estimado = 0;
        double eFulcrum = 0;
};
/*void computeForceControl(std::vector<double> sensor, double tool,
                                 double p_anterior, double Fdes, double Kf,
                                 Eigen::MatrixXd T_dest, Eigen::MatrixXd T_Fp,
                                 Eigen::MatrixXd T_FpOG,
                                 PoseDesJM* Pdest,UMA_trans* tr);*/