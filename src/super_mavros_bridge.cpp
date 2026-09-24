#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <string>     

#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
class mavros_super_bridge_node{
/**!@brief Node that bridges all integration work for px4 and voxelslam to work correctly.

        @details Still in development, and continuing to make publishes and fixes as more things are found incompatible. 
        Current pipelines of super_mavros_bridge.cpp:
        
        -# super -> mavros bridge(@ref super_sub_cb, @ref state_cb,@ref supertimerCallback), quad_0/planning/pos_cmd conversion to mavros/setpoint_raw/local. publishes at 90hz, minimum is 
        2hz for mavros commands in order stay in offboard mode. Constantly publishing with @ref supertimerCallback, and reading with callbacks. 
        If last super command was less than @ref time_threshold (currently = 0.4) then it publishes the last super command over and over

        -#VSLAM Odometry -> PX4 EKF bridge. After editing voxelslam.cpp to publish Odometry's covariance data(some of it anyways), and reading raw IMU data, 
        /quad_0/imu(200hz pub),  is read to dead reckon /Odometry(~10hz pub, can be tuned but still too slow), in order to meet 50hz reccomended 
        EKF odometry publishes (to topic :mavros/odometry/out). this is a special plugin topic, converts to mavlink's px4 odom version itself
        
        **/

public:
        

        
        mavros_super_bridge_node(ros::NodeHandle& nh){

                //-------SUPER-> MAVROS PUBS AND SUBS--------
                last_command_time = ros::Time::now();
                state_sub = nh.subscribe("mavros/state", 1,  &mavros_super_bridge_node::state_cb, this);
                super_sub = nh.subscribe("quad_0/planning/pos_cmd", 1,  &mavros_super_bridge_node::super_sub_cb, this);

                bridge_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);
                timer = nh.createTimer(ros::Duration(0.0111), &mavros_super_bridge_node::supertimerCallback,this); 
                ///super runs at 100hz, but this gon run , pubish at 90 to give a lil gap 

                // //-------ODOM & IMU -> PX4 ODOM PUBS AND SUBS--------
                // fast_odom_sub = nh.subscribe("/Odometry",10,&mavros_super_bridge_node::fast_odom_subcb, this);
                // fast_IMU_sub = nh.subscribe("/quad_0/imu",100,&mavros_super_bridge_node::fast_IMU_subcb, this);
                // fast_odom_pub = nh.advertise<nav_msgs::Odometry>("mavros/odometry/out",5);
                // odom_fast_timer = nh.createTimer(ros::Duration(0.02), &mavros_super_bridge_node::odom_fast_timerCallback,this); 



        };



private: 

        const uint16_t MASK = 0;
        mavros_msgs::State curr_state;
        //curr mav_target is still a quadrotor_msgs /Positon Command, not translated or verified yet
        quadrotor_msgs::PositionCommand::ConstPtr super_target; 
        // last mav_target is translated to mavros_msgs/PositionTarget    
        mavros_msgs::PositionTarget Last_Target; 
        bool last_target_up = false;
        mavros_msgs::PositionTarget mav_target;
        ros::Subscriber state_sub;
        ros::Subscriber super_sub;
        ros::Publisher bridge_pub;
        ros::Timer timer; 
        const float time_threshold = 0.4; //in seconds, min pub time is every .5 seconds for offboard to keep
        ros::Time last_command_time;



    //     ros::Subscriber fast_odom_sub;
    //     ros::Subscriber fast_IMU_sub;
    //     nav_msgs::Odometry::Ptr latest_odom;
    //     ros::Time last_odom_time;
    //     std::deque<boost::shared_ptr<const Eigen::Matrix3d>> R_deque;
    //     double cov_gyr=0.1; // will set by yaml later, shoudl be the same as the voxelslam'sone, represents measurement nose
    //     double cov_acc = 0.1;
    //     Eigen::Quaterniond q_imu_prev;
    // void fast_odom_subcb(const nav_msgs::Odometry::Ptr& msg){
        
    //     latest_odom = msg;
    //     //update the deque so that the only two things in it are ConstPtrs to 2 recent rotation matrices
    //     Eigen::Quaterniond quat(msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z);
    //     q_imu_prev = quat;//possibly make this a shared ptr
    //     Eigen::Matrix3d R = quat.toRotationMatrix();
    //     last_odom_time = ros::Time::now();

        
    //     R_deque.push_front(boost::make_shared<const Eigen::Matrix3d>(R));
    //     if(R_deque.size() > 2){
    //     R_deque.pop_back();
    //     }
    // }
    // void fast_IMU_subcb(const sensor_msgs::Imu::ConstPtr& msg){
    //     if (latest_odom){

    //         //latest od = k-1
    //         //msg = k
    //         //odom publishes pose as global
    //         //odom publishes twist as relative
    //         float dt = (ros::Time::now() - last_odom_time).toSec();
    //         last_odom_time = ros::Time::now();
            
    //         latest_odom->header.stamp = last_odom_time;
    //         latest_odom->header.frame_id = "map"; //maybe odom
    //         latest_odom->child_frame_id  = "base_link";
    //         //assumes sensor_msgs::Imu being printed is normalized, perfect quaternion
    //         //assumes sensor_msgs::Imu is local frame, and that R turns it local->global, 
    //         //and R.T*R[k-1]] is is like R from this frame from the last frame

    //         //uh lets hope im not wrong, or r and use of r will change everywehre
    //         //orientation is the easiest way to determine change in frame, if it works super assly then try someting like acceleration and use 
    //         //angles in quaterniond instead, willbe annoyinger though
    //         //incoming orientation, is in local frame(vslam's frame)
    //         Eigen::Quaterniond q_imu_now(msg->orientation.w, msg->orientation.x,
    //                                     msg->orientation.y, msg->orientation.z); 
    //         //use the last one to find how much orientation shifted since last time, kind of like doing R[k].T * R[k-1]
    //         Eigen::Quaterniond dq = q_imu_prev.conjugate() * q_imu_now;  // dq = change from last q to this one during dt
  
    //         //imu_prev and imu now are both from msg, both LOCAL frame. this dq doesnt have any "frame" is just a change bro!
    //         q_imu_prev = q_imu_now;// keep the k-1 imu as this one
            
    //         // adjusting the current one by dq
    //         //we should assume latest_odom is in the right frame, and q_imu changes it by exacrtly delta q
    //         Eigen::Quaterniond q_prev(latest_odom->pose.pose.orientation.w,
    //                                 latest_odom->pose.pose.orientation.x,
    //                                 latest_odom->pose.pose.orientation.y,
    //                                 latest_odom->pose.pose.orientation.z);
    //         Eigen::Quaterniond q_new = (q_prev * dq).normalized();

    //         latest_odom->pose.pose.orientation.w = q_new.w();
    //         latest_odom->pose.pose.orientation.x = q_new.x();
    //         latest_odom->pose.pose.orientation.y = q_new.y();
    //         latest_odom->pose.pose.orientation.z = q_new.z();

    //         Eigen::Matrix3d R = q_new.toRotationMatrix();   // use this R everywhere below, turns local to global
            
    //         R_deque.push_front(boost::make_shared<const Eigen::Matrix3d>(R));
    //         //deque either size 1,2,3
    //         if(R_deque.size() > 2){
    //             R_deque.pop_back();
    //         }
    //         // if deque is size 3, the last one get dropped, only two remain
    //         //if 2 of them, its fine, if 0 or 1 its fine
    //         // global linear acc = R * local - gravity
    //         Eigen::Map<Eigen::Vector3d> local_linear_accel(&msg->linear_acceleration.x);
    //         Eigen::Vector3d global_linear_accel = R * local_linear_accel + Eigen::Vector3d(0,0,-9.81);


    //         //local linear velocity, twists are in local frame v[k] = a[k] * k + R[k].T*R[k-1] * v[k-1]
    //         Eigen::Map<Eigen::Vector3d>  local_linear_veloc(&latest_odom->twist.twist.linear.x);
            
    //         local_linear_veloc = local_linear_accel*dt + R.transpose()* (*R_deque.back()) * local_linear_veloc;

    //         Eigen::Map<Eigen::Vector3d>  position(&latest_odom->pose.pose.position.x);
    //         //v_global[k] = a_global[k] * dt + R * v_local[k-1]
    //         Eigen::Vector3d global_linear_vel = global_linear_accel * dt + R*local_linear_veloc;
    //         //pos =v * dt + .5*a*dt^2 
    //         position = global_linear_vel * dt + 0.5 * global_linear_accel * dt * dt;

    //         latest_odom->twist.twist.angular.x =msg->angular_velocity.x;
    //         latest_odom->twist.twist.angular.y =msg->angular_velocity.y;
    //         latest_odom->twist.twist.angular.z =msg->angular_velocity.z;


    //         Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> pcov(latest_odom->pose.covariance.data());
    //         Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>> tcov(latest_odom->twist.covariance.data());
    //         pcov = (R.transpose() * pcov.block<3,3>(0,0)).diagonal() * R;
    //         tcov = (R.transpose() * tcov.block<3,3>(0,0)).diagonal() * R;

    //         pcov.block<3,3>(0,0).diagonal() = (R.transpose() * pcov.block<3,3>(0,0) * R).diagonal() + cov_gyr * dt * dt;
    //         pcov.block<3,3>(3,3).diagonal() = (R.transpose() * pcov.block<3,3>(3,3) * R).diagonal() + cov_gyr * dt * dt;
    //         tcov.block<3,3>(3,3).diagonal() = (R.transpose() * tcov.block<3,3>(3,3) * R).diagonal()+ cov_gyr * dt * dt;
    //         tcov.block<3,3>(0,0).diagonal() = (R.transpose() * tcov.block<3,3>(0,0) * R).diagonal() + cov_gyr * dt * dt;

    //         //assumes that the drift in twist and pose is the same, which is not true, one is instanenous and one gets refreshed 
    //         //but its fine, just need to add covariance in between /Odometry refreshes, not worth overthinking tbh


            
    //         fast_odom_pub.publish(latest_odom);
    //     }
    // }


 



    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        curr_state = *msg;
    };

    void super_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg){

        super_target = msg; //just copying the constptr, not actually value copy
        
        last_command_time = ros::Time::now();

    };





    void supertimerCallback(const ros::TimerEvent&){

// if (!super_target) {
// ROS_INFO("i null");}

// Check the threshold condition
// bool is_under_threshold = (ros::Time::now() - last_command_time).toSec() < time_threshold;

// // Print the boolean result as text
// ROS_INFO("Within time threshold: %s", is_under_threshold ? "true" : "false");

        if ((super_target) &&   ((ros::Time::now() - last_command_time).toSec() < time_threshold)) { //might not need the time threshold
                mav_target.header.stamp = ros::Time::now();
                mav_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                
                mav_target.type_mask = MASK;  
                mav_target.position.x = super_target->position.x;
                mav_target.position.y = super_target->position.y;
                mav_target.position.z = super_target->position.z;

                mav_target.velocity.x = super_target->velocity.x;
                mav_target.velocity.y = super_target->velocity.y;
                mav_target.velocity.z = super_target->velocity.z;

                mav_target.acceleration_or_force.x = super_target->acceleration.x;
                mav_target.acceleration_or_force.y = super_target->acceleration.y;
                mav_target.acceleration_or_force.z = super_target->acceleration.z;

                mav_target.yaw = super_target->yaw;
                mav_target.yaw_rate = super_target->yaw_dot;



                
                bridge_pub.publish(mav_target);
            Last_Target = mav_target;
            last_target_up = true; 
        //if last target up then we can use it, if not then just dont publish
        } else if (last_target_up){
        
            bridge_pub.publish(Last_Target);
        }


        
    };


};



int main(int argc, char** argv){
        //rosnode list will show bridge_node
        ros::init(argc, argv, "bridge_node");

        ros::NodeHandle bridge_node;
        
        mavros_super_bridge_node bridge_ob(bridge_node);

        ros::spin();
   return 0; 


}

