#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

#include "tour_robot/NextNodePlan.h"
#include "tour_robot/CartesianOdom.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "tour_robot/grid_deltaAction.h"

#include <vector>
#include <stdlib.h>
#include <math.h>

#define DEBUG true
#define HZ_ROS 10
#define OBSTCL_OFFST 0.2f
#define ACTION_TIMEOUT 30.0f

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
    ros::ServiceClient next_node_srv = n.serviceClient<tour_robot::NextNodePlan>("next_node");
    ros::ServiceClient trikh_odom_srv = n.serviceClient<tour_robot::CartesianOdom>("current_location");


    /* Action subscription */
    actionlib::SimpleActionClient<tour_robot::grid_deltaAction> grid_exec_ac("trikh_grid_exec", true);
    if(DEBUG) ROS_INFO("Waiting for trikh_grid_exec server.");
    grid_exec_ac.waitForServer();
    // Definition of goal.
    

    /*
        Main ROS loop.
        This loop acts as the coordinating layer between the reactive and
        deliberative layers in the architecture.
    */
   // general use variables
   bool timeout_flag;
   while(ros::ok())
   {
       // Call next_node service
       tour_robot::NextNodePlan next_node_rsp;
       if(!next_node_srv.call(next_node_rsp))
       {
           // In case there is an error calling the service.
           if(DEBUG) ROS_INFO("Service call to next_node has failed.");
           return(1);
       }
       else
       {
            // check if there is a next place to go.
            if(next_node_rsp.response.step_available)
            {
                // We can follow to next goal

                // we can call our next step.
                
                // We obtain the current location of the robot
                tour_robot::CartesianOdom curr_loc;
                tour_robot::grid_deltaGoal goal;
                // we request the current location service:
                if(trikh_odom_srv.call(curr_loc))
                {
                   
                    // we got a good response
                    // NOTE: We asume it impossible to have to move left and right at the same time
                    if(next_node_rsp.response.x > curr_loc.response.x )
                    {
                        // move to the left.
                        goal.delta_x = 1;
                    }
                    else
                    {
                        // move to the right.
                        goal.delta_x = -1;
                    }
                    if(next_node_rsp.response.y > curr_loc.response.y)
                    {
                        // move up
                        goal.delta_y = 1;
                    }
                    else
                    {
                        // move down;
                        goal.delta_y = -1;
                    }

                    // we have calculated the direction to go next.
                }
                else
                {
                    // erroneous response
                    if(DEBUG) ROS_INFO("Service call \"current_location\" returned error.");
                    return(1);
                }

                // send the goal.
                grid_exec_ac.sendGoal(goal);

                // wait for goal to finish.
                timeout_flag = grid_exec_ac.waitForResult(ros::Duration(ACTION_TIMEOUT));

                // at this point we should have moved to the new location.
                if(DEBUG) ROS_INFO("We have a next call.");
            }
       }
       

       // Do a spin.
       ros::spinOnce();
       loop_rate.sleep();
   }
    

}