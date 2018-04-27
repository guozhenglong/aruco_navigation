#include "localization.h"
namespace Localization
{
    local_position::local_position(ros::NodeHandle& nh,ros::NodeHandle& pnh):nh_(nh)
    {

        pnh.param("debug", debug, true);

        pnh.param("Lkp", Lkp, 1.0);
        pnh.param("Lkq", Lkq, 1.0);
        pnh.param("Lkr", Lkr, 1.0);

        pnh.param("low_pass_param_vel", low_pass_param_vel, 0.3);
        pnh.param("Hz", Hz, 20.0);

        local_position::KalmanFilterInit();

        get_marker_pose = nh_.subscribe("/aruco_eye/aruco_observation",1,&local_position::MarkerPoseCallback,this);
        att_uav = nh_.advertise<geometry_msgs::PointStamped>("/att_uav",1);

        pos_uav = nh_.advertise<geometry_msgs::PoseStamped>("/pos_uav",1);
        pos_comp_uav = nh_.advertise<geometry_msgs::PoseStamped>("/comp_pos_uav", 1);
        pos_uav_kf = nh_.advertise<geometry_msgs::PoseStamped>("/pos_uav_kf",1);
        vel_uav = nh_.advertise<geometry_msgs::PointStamped>("/vel_uav",1);
        vel_uav_kf = nh_.advertise<geometry_msgs::PointStamped>("/vel_uav_kf", 1);

        navdata_pub = nh_.advertise<aruco_navigation::Navdata_aruco>("/navdata_uav", 1);

        ros::Rate loopRate(Hz);
        pos_time_last = ros::Time::now();
        while(ros::ok())
        {
            
            att_uav.publish(att_pub);

            pos_uav.publish(pos_pub);
            pos_uav_kf.publish(pos_kf_pub);
            vel_uav.publish(vel_pub);
            vel_uav_kf.publish(vel_kf_pub);

            pos_comp_uav.publish(pos_comp_pub);

            navdata_pub.publish(navdata_msg);

            ros::spinOnce();
            loopRate.sleep();
        }
    }

    void local_position::KalmanFilterInit()
    {
        rot = Eigen::Matrix3d::Zero(3,3);  
        I_6 = Eigen::MatrixXd::Zero(6,6);
        I_3 = Eigen::MatrixXd::Zero(3,3);
        F = Eigen::MatrixXd::Zero(6,6);
        H = Eigen::MatrixXd::Zero(3,6);
        Tao = Eigen::MatrixXd::Zero(6,6);


        Lx_e_= Eigen::VectorXd::Zero(6);
        Lx_e= Eigen::VectorXd::Zero(6);
        LP_ = Eigen::MatrixXd::Zero(6,6);
        LP = Eigen::MatrixXd::Zero(6,6);
        LQ = Eigen::MatrixXd::Zero(6,6);
        LR = Eigen::MatrixXd::Zero(3,3);
        LK = Eigen::MatrixXd::Zero(6,6);

        
        dt = 1.0/Hz;
        F<< 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
        //F = dt*F + I_6;

        Tao<< 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

        H<< 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0;

        I_6<<1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
       
        I_3<< 1.0, 0.0, 0.0, 
            0.0, 1.0, 0.0, 
            0.0, 0.0, 1.0; 
  
        Lx_e<< 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; 
        LP = Lkp * I_6;
        LQ = Lkq * I_6;
        LR = Lkr * I_3;

        // position_last and velocity_last zeros.
        position_last.x = 0.0;
        position_last.y = 0.0;
        position_last.z = 0.0;
      
        velocity_last.x = 0.0;
        velocity_last.y = 0.0;
        velocity_last.z = 0.0;

        velocity_dir.x = 0.0;
        velocity_dir.y = 0.0;
        velocity_dir.z = 0.0;

    }


    void local_position::PositionKalmanFilter(geometry_msgs::Point& position, Eigen::VectorXd& Lx_e)
    {
         // Kalman Filter
         Lz_m(0) = position.x;
         Lz_m(1) = position.y;
         Lz_m(2) = position.z;
         Lx_e_ = (dt*F + I_6) * Lx_e ;
         LP_   =(dt*F + I_6) * LP * (dt*F + I_6).transpose() + Tao * LQ * Tao.transpose();    
         LK    = LP_ * H.transpose() * (H * LP_ * H.transpose() + LR).inverse();
         Lx_e  = Lx_e_ + LK * (Lz_m - H * Lx_e_);
         LP    = (I_6 - LK * H) * LP_;
        //  Lx_e(2) =  Lx_e(2)>min_height ?min_height:Lx_e(2); //
        //  Lx_e(2) =  Lx_e(2)<max_height ?max_height:Lx_e(2);  //height < 0

    }

    void local_position::MarkerPoseCallback(const aruco_eye_msgs::MarkerList& msg)
    {
               
        if(!(msg.markers.empty()))
        {
            ros::Time time_stamped = msg.header.stamp;
            count_markers = msg.markers.size();
            position_dir.x = 0.0; 
            position_dir.y = 0.0; 
            position_dir.z = 0.0; 
            position_comp.x = 0.0; 
            position_comp.y = 0.0; 
            position_comp.z = 0.0; 
            
            roll = 0.0;
            pitch = 0.0;
            yaw = 0.0;


            // calculate the attitude
            for(int i=0;i<count_markers;i++)
            {
                quat_marker =  msg.markers[i].pose.pose.orientation;
                // The camera coordinate is different from the tag coordiante ,
                //so the quaternion includes this transform and we need to compensate it. 
                //local_position::Quat_ComPen_Cam2Tag(quat_marker);
                local_position::Quat2Euler(quat_marker,rpy);
                roll  += rpy.x;
                pitch += rpy.y;
                yaw   += rpy.z; 
            }
            roll  /= count_markers;
            pitch /= count_markers;
            yaw   /= count_markers;
            yaw = - yaw; // the image is opposite to the real scene. The left in image is the right in bebop, the right in image is the left in bebop.
    
            att_pub.header.frame_id = "bebop_att";
            att_pub.header.stamp = time_stamped;
            att_pub.point.x = roll;
            att_pub.point.y = pitch;
            att_pub.point.z = yaw;

            euler.x = roll;
            euler.y = pitch;
            euler.z = yaw;              
            local_position::Euler2Quat(euler, quat_pub);
            if (debug)
            {
                cout << "Roll =:" << euler.x << "     Pitch =:" << euler.y << "     Yaw =:" << euler.z << endl;
            }

            // calculate the position and velocity
            for(int i=0;i<count_markers;i++)
            {
               
                pos_marker = msg.markers[i].pose.pose.position;
                ID = msg.markers[i].id;
                //marker.x point to the y axis of body coordinate; marker.y point to the -x axis
                //tags coordinate is opposite to the body coordinate, when the front of ardrone is point north

                pos_in_body(0) = (- pos_marker.y);
                pos_in_body(1) = pos_marker.x;
                pos_in_body(2) = pos_marker.z;

                position_dir.x += (-pos_in_body(0) + (ID/20)*dx); 
                position_dir.y += (-pos_in_body(1) + (ID%20)*dy);
                position_dir.z += (-pos_in_body(2)); 

                //local_position::Euler2Rota(euler,rot);
                //Quat2Rota(quat_ave,rot);
                //pos_in_tag = rot*pos_in_body;

                rot(0, 0) = -cos(euler.y);
                rot(0, 1) = 0;
                rot(0, 2) = -sin(euler.y);
                rot(1, 0) = 0;
                rot(1, 1) = -cos(euler.x);
                rot(1, 2) = sin(euler.x);
                rot(2, 0) = -sin(euler.y);
                rot(2, 1) = sin(euler.x);
                rot(2, 2) = cos(euler.x) * cos(euler.y);
                pos_in_tag = rot * (-pos_in_body);

                position_comp.x += (pos_in_tag(0) + (ID/20)*dx);
                position_comp.y += (pos_in_tag(1) + (ID%20)*dy);
                position_comp.z += (pos_in_tag(2));    
            }

            position_dir.x /= count_markers;
            position_dir.y /= count_markers;
            position_dir.z /= count_markers;
            pos_pub.header.frame_id = "bebop_pos";
            pos_pub.header.stamp = time_stamped;
            pos_pub.pose.position = position_dir;
            pos_pub.pose.orientation = quat_pub;
            pos_pub.pose.orientation.w = yaw;
            if (debug)
            {
                cout<<"=============CURRENT POS =========================="<<endl;
                cout<<" current x:"<<position_dir.x<<endl;
                cout<<" current y:"<<position_dir.y<<endl;
                cout<<" current z:"<<position_dir.z<<endl;
                cout<<" current yaw:"<<yaw<<endl;
            }

            velocity_dir.x = (position_dir.x - position_last.x)/(time_stamped-pos_time_last).toSec();
            velocity_dir.y = (position_dir.y - position_last.y)/(time_stamped-pos_time_last).toSec();
            velocity_dir.z = (position_dir.z - position_last.z)/(time_stamped-pos_time_last).toSec();
            velocity_dir.x = low_pass_param_vel * velocity_last.x + (1-low_pass_param_vel) * velocity_dir.x;
            velocity_dir.y = low_pass_param_vel * velocity_last.y + (1-low_pass_param_vel) * velocity_dir.y;
            velocity_dir.z = low_pass_param_vel * velocity_last.z + (1-low_pass_param_vel) * velocity_dir.z;

            vel_pub.header.frame_id="bebop_vel";
            vel_pub.header.stamp = time_stamped;
            vel_pub.point = velocity_dir;

            position_last = position_comp;
            velocity_last = velocity_dir;
            pos_time_last = time_stamped;


            local_position::PositionKalmanFilter(position_comp, Lx_e);
            pos_kf_pub.pose.position.x = Lx_e(0);
            pos_kf_pub.pose.position.y = Lx_e(1);
            pos_kf_pub.pose.position.z = Lx_e(2);
            pos_kf_pub.header.frame_id = "bebop_pos_kf";
            pos_kf_pub.header.stamp = time_stamped;
            pos_kf_pub.pose.orientation = quat_pub;

            vel_kf_pub.header.frame_id="bebop_vel_kf";
            vel_kf_pub.header.stamp = time_stamped;
            vel_kf_pub.point.x = Lx_e(3);
            vel_kf_pub.point.x = Lx_e(4);
            vel_kf_pub.point.x = Lx_e(5);

            position_comp.x /= count_markers;
            position_comp.y /= count_markers;
            position_comp.z /= count_markers;
            pos_comp_pub.header.frame_id = "bebop_pos_comp";
            pos_comp_pub.header.stamp = time_stamped;
            pos_comp_pub.pose.position = position_comp;
            pos_comp_pub.pose.orientation = quat_pub;

            navdata_msg.header.frame_id = "navdata_all";
            navdata_msg.header.stamp = time_stamped;
            navdata_msg.pose = pos_kf_pub.pose;
            navdata_msg.velocity = vel_pub.point;
            navdata_msg.euler_angle = euler;
        }
        else
             if (debug)
             {
                 ROS_INFO("No tag pose message!");
             }
            
    }

    void local_position::Quat2Euler(geometry_msgs::Quaternion &quat, geometry_msgs::Vector3 &euler)
    {
        double q0 = quat.w;
        double q1 = quat.x;
        double q2 = quat.y;
        double q3 = quat.z;
    
        double t0 = -2.0 * (q2 * q2 + q3 * q3) + 1.0;
        double t1 = +2.0 * (q1 * q2 + q0 * q3);
        double t2 = -2.0 * (q1 * q3 - q0 * q2);
        double t3 = +2.0 * (q2 * q3 + q0 * q1);
        double t4 = -2.0 * (q1 * q1 + q2 * q2) + 1.0;
    
        // t2 = t2 > 1.0 ? 1.0 : t2;
        // t2 = t2 < -1.0 ? -1.0 : t2; 
        euler.x = asin(t2);
        euler.y = -atan2(t3, t4);
        euler.z = atan2(t1, t0);
    }
    
    void local_position::Euler2Quat(geometry_msgs::Vector3 &euler,geometry_msgs::Quaternion &quat)
    {
        double fi  = euler.x / 2;
        double theta = euler.y / 2;
        double psi   = euler.z / 2;
        
        quat.w = cos(fi)*cos(theta)*cos(psi) + sin(fi)*sin(theta)*sin(psi);  
        quat.x = sin(fi)*cos(theta)*cos(psi) - cos(fi)*sin(theta)*sin(psi);
        quat.y = cos(fi)*sin(theta)*cos(psi) + sin(fi)*cos(theta)*sin(psi);
        quat.z = cos(fi)*cos(theta)*sin(psi) - sin(fi)*sin(theta)*cos(psi);
    }
    
    void local_position::Quat2Rota(geometry_msgs::Quaternion &quat, Eigen::Matrix3d& rot)
    {
        double x = quat.x;
        double y = quat.y;
        double z = quat.z;
        double w = quat.w;
    
        rot(0,0)=1-2*(y*y+z*z);
        rot(0,1)=2*(x*y-z*w);
        rot(0,2)=2*(x*z+y*w);
    
        rot(1,0)=2*(x*y+z*w);
        rot(1,1)=1-2*(x*x+z*z);
        rot(1,2)=2*(y*z-x*w);
    
        rot(2,0)=2*(x*z-y*w);
        rot(2,1)=2*(y*z+x*w);
        rot(2,2)=1-2*(x*x+y*y);
    
    }
    
    void local_position::Quat_ComPen_Cam2Tag(geometry_msgs::Quaternion& q)
    {
        //cam coordinate and tag coordinate: tag coordinate is rotated -90 deg about the x axis of cam coordinate.
        // compensate the rotation, so rotate 90 deg. angle_compensate = 90, cos(angle_compensate/2) = sin(angle_compensate/2) = 0.707
        // x = 1*sin(angle_compensate/2), y = 0, z = 0, w = cos(angle_compensate/2);
        // quaternion1 * quaternion2 = " you can google ! " 
        
        double x = q.x;
        double y = q.y;
        double z = q.z;
        double w = q.w;
        q.x = 0.707 * (x + w);
        q.y = 0.707 * (y + z);
        q.z = 0.707 * (z - y);
        q.w = 0.707 * (w - x);
    }
    
    void local_position::Euler2Rota(geometry_msgs::Vector3 &euler, Eigen::Matrix3d& rot)
    {
        double fi = euler.x;
        double theta = euler.y;
        double psi = euler.z;
        Eigen::Matrix3d Rx(3,3);
        Eigen::Matrix3d Ry(3,3);
        Eigen::Matrix3d Rz(3,3);
        Rz<<cos(psi), sin(psi), 0.0,
            -sin(psi), cos(psi), 0.0,
            0.0, 0.0, 1.0;   
        Ry<<cos(theta), 0.0, -sin(theta),
            0.0, 1.0, 0.0,
            sin(theta), 0.0, cos(theta);
        Rx<<1.0, 0.0, 0.0,
            0.0, cos(fi), sin(fi),
            0.0, -sin(fi), cos(fi);
        rot = Rz*Ry*Rx;
    }

}