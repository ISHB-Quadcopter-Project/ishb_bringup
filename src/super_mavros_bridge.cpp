#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <quadrotor_msgs/PositionCommand.h>
// Global variable to store vehicle status
mavros_msgs::State current_state;
int staleness_threshold = 5;
int last_command_time_ ros::Time::now();
mavros_msgs::PositionTarget target;
mavros_msgs::PositionTarget last_target;
uint16 MASK = 0; //nothgn masked out

// Callback function (similar to your Python callback)
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
/// @brief A short description of the function.
/// @param x The first input coordinate.
/// @return True if successful, false otherwise.

// terrible job, gotta make classses 
void super_sub_cb(const quadrotor_msgs::PositionCommand::ConstPtr& msg){

    target.header.stamp = ros::Time::now();
    target.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    last_command_time_ = ros::Time::now();
    
    target.type_mask = MASK;  
    target.position.x = msg->position.x;
    target.position.y = msg->position.y;
    target.position.z = msg->position.z;

    target.velocity.x = msg->velocity.x;
    target.velocity.y = msg->velocity.y;
    target.velocity.z = msg->velocity.z;

    target.acceleration_or_force.x = msg->acceleration.x;
    target.acceleration_or_force.y = msg->acceleration.y;
    target.acceleration_or_force.z = msg->acceleration.z;

    target.yaw = msg->yaw;
    target.yaw_rate = msg->yaw_dot;
    
}





int main(int argc, char **argv)
{
    ros::init(argc, argv, "offboard_node");
    ros::NodeHandle nh;

    // Subscriber and Publisher
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    /// subscribe to super command messages
    ros::Subscriber super_sub = nh.subscribe<quadrotor_msgs::PositionCommand>
            ("/planning/pos_cmd", 100,super_sub_cb);
            
   
    ros::Publisher bridge_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("setpoint_raw/local", 10);


            
    // Clients for Services (Arming and Mode switching)
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");



    ros::Timer timer = nh.createTimer(ros::Duration(0.0111), timerCallback); 
    ///super runs at 100hz, but this gon run , pubish at 90 to give a lil gap 
    return 0;
}
