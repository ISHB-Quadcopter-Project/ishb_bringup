
    #include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <string>       


class mavros_super_bridge_node{


public:

        mavros_super_bridge_node(ros::NodeHandle& nh){ 
                last_command_time = ros::Time::now();
                state_sub = nh.subscribe("mavros/state", 1,  &mavros_super_bridge_node::state_cb, this);
                super_sub = nh.subscribe("quad_0/planning/pos_cmd", 1,  &mavros_super_bridge_node::super_sub_cb, this);
                bridge_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);
                timer = nh.createTimer(ros::Duration(0.0111), &mavros_super_bridge_node::timerCallback,this); 
                ///super runs at 100hz, but this gon run , pubish at 90 to give a lil gap 


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
   

    void state_cb(const mavros_msgs::State::ConstPtr& msg){
        curr_state = *msg;
    };

    void super_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg){

        super_target = msg; //just copying the constptr, not actually value copy
        
        last_command_time = ros::Time::now();

    };

    void timerCallback(const ros::TimerEvent&){
//sample trajectory
                // mav_target.header.stamp = ros::Time::now();
                // mav_target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
                // mav_target.type_mask = MASK;  
                // mav_target.position.x = 1;
                // mav_target.position.y = 5;
                // mav_target.position.z = 2;

                // mav_target.velocity.x = 0;
                // mav_target.velocity.y = 0;
                // mav_target.velocity.z = 0;

                // mav_target.acceleration_or_force.x = 0;
                // mav_target.acceleration_or_force.y = 0;
                // mav_target.acceleration_or_force.z = 0;

                // mav_target.yaw = 0;
                // mav_target.yaw_rate = 2;
                // bridge_pub.publish(mav_target);



        // check if super target null and if stale


// Ensure super_target is converted to a C-string if it is a std::string

// if (!super_target) {
// ROS_INFO("i null");}

// Check the threshold condition
// bool is_under_threshold = (ros::Time::now() - last_command_time).toSec() < time_threshold;

// // Print the boolean result as text
// ROS_INFO("Within time threshold: %s", is_under_threshold ? "true" : "false");

        if ((super_target) &&   ((ros::Time::now() - last_command_time).toSec() < time_threshold)) {
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

