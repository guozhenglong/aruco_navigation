#include <iostream>
#include <ros/ros.h>
#include <stdio.h>
#include <math.h>
#include <cstdlib>
#include <stdlib.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> 

 //ROS Images
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <aruco_navigation/Navdata_aruco.h>

 //Aruco Messages
#include "aruco_eye_msgs/PointInImage.h"
#include "aruco_eye_msgs/Marker.h"
#include "aruco_eye_msgs/MarkerList.h"
        
#define dx 0.196
#define dy 0.196

using namespace std;

namespace Localization{
class local_position{
    public:
        local_position(ros::NodeHandle& nh,ros::NodeHandle& pnh);
        void MarkerPoseCallback(const aruco_eye_msgs::MarkerList& msg);
        void Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Vector3 &euler);
        void Euler2Quat(geometry_msgs::Vector3 &euler,geometry_msgs::Quaternion &quat);
        void Quat2Rota(geometry_msgs::Quaternion &quat, Eigen::Matrix3d& rot);
        void Quat_ComPen_Cam2Tag(geometry_msgs::Quaternion& q);
        void Euler2Rota(geometry_msgs::Vector3 &euler, Eigen::Matrix3d& rot);
        void KalmanFilterInit();
        void PositionKalmanFilter(geometry_msgs::Point& position, Eigen::VectorXd& Lx_e);
    private:
        bool debug;
        ros::NodeHandle nh_;
        ros::Subscriber get_marker_pose ;
        ros::Publisher att_uav;
        ros::Publisher pos_uav;
        ros::Publisher pos_comp_uav;
        ros::Publisher pos_uav_kf ;
        ros::Publisher vel_uav;
        ros::Publisher vel_uav_kf;
        ros::Time pos_time_last;

        ros::Publisher navdata_pub;
        aruco_navigation::Navdata_aruco navdata_msg;

        geometry_msgs::PointStamped  att_pub;
        geometry_msgs::Quaternion quat_pub;

        geometry_msgs::PoseStamped  pos_pub;
        geometry_msgs::PoseStamped  pos_comp_pub;    
        geometry_msgs::PoseStamped  pos_kf_pub;

        geometry_msgs::PointStamped  vel_pub;
        geometry_msgs::PointStamped  vel_kf_pub;

        geometry_msgs::Point position_dir;
        geometry_msgs::Point velocity_dir;
        geometry_msgs::Point position_comp;

        geometry_msgs::Point position_last;
        geometry_msgs::Point velocity_last;

        int count_markers, ID;
        double roll, pitch, yaw;
        double min_height = -0.8;
        double max_height = -2.5;
        double Hz;

        geometry_msgs::Point pos_marker;
        geometry_msgs::Quaternion quat_marker;

        geometry_msgs::Vector3 euler;
        geometry_msgs::Vector3 rpy;
        
        Eigen::Matrix3d rot;
        Eigen::Vector3d pos_in_body;
        Eigen::Vector3d pos_in_tag;




        double low_pass_param_vel= 0.3; // velocity low pass filter parameter
        
        // for kalman filter, x -> pos, z -> measurement
        double dt;
        Eigen::MatrixXd I_6;
        Eigen::Matrix3d I_3;
        Eigen::MatrixXd F;
        Eigen::MatrixXd Tao;
        Eigen::MatrixXd H;

        //for position 
        double Lkp, Lkq, Lkr;
        Eigen::VectorXd Lx_e_;
        Eigen::VectorXd Lx_e;
        Eigen::Vector3d Lz_m;
        Eigen::MatrixXd LP_;
        Eigen::MatrixXd LP;
        Eigen::MatrixXd LQ;
        Eigen::Matrix3d LR;
        Eigen::MatrixXd LK;


};
}






