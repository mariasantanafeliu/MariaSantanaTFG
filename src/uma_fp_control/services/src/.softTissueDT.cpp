//class to simulate a soft tissue. The inputs are the pose vector and th Z position of a plane. The output is a simulated force
#include <iostream>
#include <vector>
#include "ros/ros.h"
#include <Eigen/Dense>
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
class softTissue {
    private:
        ros::NodeHandle nh_;
        ros::Subscriber pose_sub_;
        ros::Publisher force_pub_;
        ros::Publisher forceS_pub_;
        //variables to be published an std::vector<double>
        geometry_msgs::Point force_msg_;
        geometry_msgs::Point forceS_msg_;
        Eigen::MatrixXd T_;
        //coordinates
        std::vector<double> currentCoordinates_;
        std::vector<double> contactCoordinates_;
        std::vector<double> distVector_;
        double z_Plane_;
        double Kviscoelastica_;
    public:
        softTissue(double zPlane, double cteViscoelastica): z_Plane_(zPlane), Kviscoelastica_(cteViscoelastica){
            pose_sub_ = nh_.subscribe("/bob/pose_topic", 1000, &softTissue::cmd_pose, this);
            force_pub_ = nh_.advertise<geometry_msgs::Point>("force_topic", 1000);
            forceS_pub_ = nh_.advertise<geometry_msgs::Point>("forceS_topic", 1000);
            distVector_ = {0, 0, 0};
            currentCoordinates_ = {0, 0, 0};
            contactCoordinates_ = {0, 0, 0};
        }
        bool contact_ = false;
        bool lastContact_ = false;
        //callback
        //calback cmd_pose
        void cmd_pose(const std_msgs::Float64MultiArray::ConstPtr& msg){
            Eigen::MatrixXd matrix(4, 4);
            int index = 0;
            for (int j = 0; j < 4; ++j) {
                for (int i = 0; i < 4; ++i) {
                    matrix(i,j) = msg->data[index++];
                }
            }
            T_ = matrix;
            getTissueForce();
        }
        //function
        std::vector<double> getTissueForce() {
            std::vector<double> force = {0, 0, 0};
            currentCoordinates_ = {T_(0,3), 0, T_(2,3)}; //por ahora solo trabajamos en el plano XZ
            if (T_(2,3) > z_Plane_) {
                contact_= false;
                contactCoordinates_ = currentCoordinates_;
            } else {
                contact_ = true;
            }
            if (lastContact_ != contact_ && contact_ == true){
                contactCoordinates_ = currentCoordinates_;
            }
            lastContact_ = contact_;
            Eigen::Matrix3d R;
            R << T_(0,0), T_(0,1), T_(0,2),
                T_(1,0), T_(1,1), T_(1,2),
                T_(2,0), T_(2,1), T_(2,2);
            for (int p = 0; p < 3; ++p) {
                distVector_[p] = currentCoordinates_[p] - contactCoordinates_[p];
                force[p] = Kviscoelastica_ * distVector_[p];
            }
            Eigen::Vector3d eigen_force(force[0], force[1], force[2]);
            Eigen::Vector3d force_s = -R * eigen_force;
            //publish force
            force_msg_.x = force[0];
            force_msg_.y = force[1];
            force_msg_.z = force[2];
            //publish forceS
            forceS_msg_.x = force_s[0];
            forceS_msg_.y = force_s[1];
            forceS_msg_.z = force_s[2];
            force_pub_.publish(force_msg_);
            forceS_pub_.publish(forceS_msg_);
            return force;
        }
};
int main(int argc, char **argv) {
    // Inicializa el nodo de ROS
    ros::init(argc, argv, "soft_tissue_simulator");
    softTissue simulator(-0.033642, 350);
    ros::spin();
    return 0;
}