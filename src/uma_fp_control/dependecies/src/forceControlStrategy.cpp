#include <dependecies/forceControlStrategy.hpp>

forceControl::forceControl(){
	//ROS_INFO_STREAM("---forceControl---");
}

forceControl::~forceControl(){
  //ROS_INFO_STREAM("Leaving gently...");
}
//void forceControl::computeForceControl(std::vector<double> sensor, double tool, double p_anterior,double Fdes, double Kf, Eigen::MatrixXd T_dest, Eigen::MatrixXd T_Fp, Eigen::MatrixXd T_FpOG,PoseDesJM* Pdest,UMA_trans* tr)
void forceControl::computeForceControl(std::vector<double> sensor, double tool, double p_anterior,double Fdes, double Kf, Eigen::MatrixXd T_Fp, Eigen::MatrixXd T_FpOG,PoseDesJM* Pdest,UMA_trans* tr)
{
  //fulcrum fp;
  
  Eigen::MatrixXd W_DespP(4,1);
  Eigen::MatrixXd R_DespP(4,1);

  //ROS_INFO_STREAM("***********------+-----***********");

  if (fabs(sensor[0])>0.2)
  {
    Xf[0]=(Fdes - sensor[0])*Kf;
  }
  else{
    Xf[0]=0;
  }
  if (fabs(sensor[1])>0.2)
  {
    Xf[1]=(Fdes - sensor[1])*Kf;
  }
  else{
    Xf[1]=0;
  }
  /*if ((fabs(sensor[0])<0.2) && (fabs(sensor[1])<0.2))
  {
    forceFulcrum.T_FpOG=T_Fp;
  }*/

  if ((Xf[0] != 0) || (Xf[1] != 0) )//|| (fabs(eFulcrum)>0.008))
  {
    //ERROR RHO
    p_estimado = 0;//fp.computeFulcrum(sensor,tool,p_anterior);
    
    // prueba cambiar p_estiamdo
    /*p_estimado = incrementalFunction(p_anterior, 0.222, 0.02);
    p_estimado = p_anterior + 0.001;
    if ((p_estimado) >= 0.222) {
      p_estimado = 0.222;
    }*/
    //ERROR DELTA_X
    ROS_INFO_STREAM("DESP FUERZA:");
    ROS_INFO_STREAM(Xf[0]);
    ROS_INFO_STREAM(Xf[1]);
    W_DespP<<Xf[0],Xf[1],0,0;
    R_DespP=T_FpOG * W_DespP;
    DespP[0]=R_DespP(0);
    DespP[1]=R_DespP(1);
    DespP[2]=R_DespP(2);
    ROS_INFO_STREAM("DESP FUERZA R:");
    ROS_INFO_STREAM(DespP[0]);
    ROS_INFO_STREAM(DespP[1]);
    ROS_INFO_STREAM(DespP[2]);
    Desp_rho = {0,0, eFulcrum};
    /*T_Fp =(tr->desp(DespP))*T_Fp *(tr->desp(Desp_rho));
    ROS_INFO_STREAM("T_Fp");
    ROS_INFO_STREAM(T_Fp);
    forceFulcrum.T_Fp = T_Fp; */
    forceFulcrum.T_Fp  =(tr->desp(DespP))*T_Fp *(tr->desp(Desp_rho));
    //forceFulcrum.TTP_dest_orientado = Pdest->computePoseDes(T_dest,T_Fp);
    //ROS_INFO_STREAM(forceFulcrum.T_Fp);
    /*forceFulcrum.RPYdest[0] = Pdest.roll_dest;
    forceFulcrum.RPYdest[1] = Pdest.pitch_dest;
    forceFulcrum.RPYdest[2] = Pdest.yaw_dest;*/
    forceFulcrum.p_anterior = p_estimado;
  } else {
    forceFulcrum.p_anterior = p_anterior;
    forceFulcrum.T_Fp = T_Fp;
  }

}