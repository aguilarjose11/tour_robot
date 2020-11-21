#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

#include "std_srvs/SetBool.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "tour_robot/grid_deltaAction.h"

#include <vector>
#include <stdlib.h>
#include <math.h>

#define DEBUG true
#define HZ_ROS 10
#define OBSTCL_OFFST 0.2f

/*
    The executive node.

    The main objective of this node is to find out what is the next command
    to execute. 

    The node will run a ros loop where the 
*/






int main(int argc, char **argv)
{
    // initialize the executive node.
    ros::init(argc, argv, "trikh_executive");
    // create node handle to have communication with other nodes.
    ros::NodeHandle n;
    // ros rate/clock of comunication.
    ros::Rate loop_rate(HZ_ROS);

    /* callback class declarations */
    // RobotReactive robot_reactive(OBSTCL_OFFST);

    /* topic Subscription */
    // motor_pub: to allow us to move the turtlebot. Uses geometry_msgs/Twist
    // ros::Publisher motor_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1000);


    /* topic Publishing */


    /* Service subscription */
    // from trikh_planner.
    ros::ServiceClient next_node_srv = n.serviceClient<std_srvs::SetBool>("next_node");


    /* Action subscription */
    actionlib::SimpleActionClient<tour_robot::grid_deltaAction> grid_exec_ac("trikh_grid_exec", true);
    if(DEBUG) ROS_INFO("Waiting for trikh_grid_exec server.");
    grid_exec_ac.waitForServer();
    // Definition of goal.
    tour_robot::grid_deltaAction goal;

    /*
        Main ROS loop.
        This loop acts as the coordinating layer between the reactive and
        deliberative layers in the architecture.
    */
   while(ros::ok())
   {
       // Call next_node service
       std_srvs::SetBool next_node_rsp;
       if(!next_node_srv.call(next_node_rsp))
       {
           // In case there is an error calling the service.
           if(DEBUG) ROS_INFO("Service call to next_node has failed.");
           return(1);
       }
       else
       {
           // we can call our next step.
           // actionlib::
           if(DEBUG) ROS_INFO("We have a next call.");
       }
       

       // Do a spin.
       ros::spinOnce();
       loop_rate.sleep();
   }
    

}