#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include "rio_control_node/Motor_Control.h"
#include "rio_control_node/Motor_Config.h"
#include "rio_control_node/Motor_Status.h"
#include "rio_control_node/Robot_Status.h""

#include <thread>
#include <string>
#include <mutex>

#define RATE (100)

ros::NodeHandle* node;
std::map<std::string, int> links_to_watch;


void gazebo_link_states_callback(const gazebo_msgs::LinkStates &msg)
{
	static ros::Publisher motor_status_pub
        = node->advertise<rio_control_node::Motor_Status>("MotorStatus", 1);

    rio_control_node::Motor_Status motor_status;

    for( size_t i = 0; i < msg.name.size(); i++)
    {
        std::string link_name = msg.name[i];
        // we are watching this link
        if( links_to_watch.count( link_name ) )
        {
            rio_control_node::Motor_Info motor_info;
            motor_info.sensor_position = msg.pose[i].position.y;
            motor_info.sensor_velocity = msg.twist[i].angular.y;
            motor_info.id = links_to_watch[link_name];
            motor_status.motors.push_back( motor_info );
        }
    }

    motor_status_pub.publish(motor_status);
}


void motor_config_callback(const rio_control_node::Motor_Config &msg)
{
    (void)msg;
}

void motor_control_callback(const rio_control_node::Motor_Control &msg)
{
    std::cout << "Motor Control Callback\n";
    for( size_t i = 0; i < msg.motors.size(); i++ )
    {
        std::string body_name;
        bool found = false;
        for( auto it = links_to_watch.begin(); it != links_to_watch.end(); it++ )
        {

            std::cout << "it: " << it->first << ":" << it->second << " ... " << msg.motors[i].id << "\n";
            if( it->second == msg.motors[i].id )
            {
                body_name = it->first;
                found = true;
                break;
            }
        }

        if( !found ){ continue; }

        std::cout << "Body : " << body_name << "\n";

        static ros::ServiceClient gazebo_body_wrench_client
            = node->serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

        gazebo_msgs::ApplyBodyWrench apply_wrench;
        apply_wrench.request.body_name = body_name;
        apply_wrench.request.wrench.torque.y = msg.motors[i].output_value;
        apply_wrench.request.duration = ros::Duration(1);

        gazebo_body_wrench_client.call( apply_wrench );
        std::cout << "Sent: " << apply_wrench.request << "\n";
    }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "flatty_light_sim_rio_control_node");
	ros::NodeHandle n;
	node = &n;
    ros::Rate rate(RATE);

    links_to_watch["flatty::wheel_l_1"] = 0;
    links_to_watch["flatty::wheel_l_2"] = 1;
    links_to_watch["flatty::wheel_l_3"] = 2;
    links_to_watch["flatty::wheel_r_1"] = 3;
    links_to_watch["flatty::wheel_r_2"] = 4;
    links_to_watch["flatty::wheel_r_3"] = 5;

	ros::Subscriber motorConfig = node->subscribe("MotorConfiguration", 100, motor_config_callback);
	ros::Subscriber motorControl = node->subscribe("MotorControl", 100, motor_control_callback);
	ros::Subscriber gazeob_link_states = node->subscribe("/gazebo/link_states", 100, gazebo_link_states_callback);


    while( ros::ok() )
    {
        ros::spinOnce();
        rate.sleep();
    }


	return 0;
}
