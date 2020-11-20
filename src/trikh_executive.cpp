#include "ros/ros.h"

#include "std_srvs/SetBool.h"

#include <vector>
#include <stdlib.h>
#include <math.h>

#define DEBUG true
#define HZ_ROS 10

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

    /* topic Subscription */


    /* topic Publishing */


    /* Service subscription */
    ros::ServiceClient next_node_srv = n.serviceClient<std_srvs::SetBool>("next_node");
    

    /* Action subscription */


    /*
        Main ROS loop.
        This loop acts as the coordinating layer between the reactive and
        deliberative layers in the architecture.
    */
   while(ros::ok())
   {
    


       // Do a spin.
       ros::spinOnce();
       loop_rate.sleep();
   }
    

}