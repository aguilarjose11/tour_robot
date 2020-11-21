#include "ros/ros.h"

#include "std_srvs/SetBool.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

#include <vector>
#include <stdlib.h>
#include <math.h>

#define DEBUG true
#define HZ_ROS 10
#define OBSTCL_OFFST 0.2f



/* function definitions */

void motorDriver(ros::Publisher pub, geometry_msgs::Twist msg)
{
    /*
    will take care of doing the movement of the wheels. will recieve the amount to be
    moved and will convert into a message and then publish to the respective topic

    Publishes to: /mobile_base/commands/velocity
    msg type: geometry_msgs/Twist
    */
   pub.publish(msg);
   return;
}


/* Class definitions */

class RobotReactive
{
    public:
        RobotReactive(double offset);
        char override();
        void obstacleCallBack(sensor_msgs::LaserScan msg);
    private:
        const double offset;
        char stimulus;
};
/* Robot Reactive member definitions. */


int main(int argc, char **argv)
{
    // initialize the executive node.
    ros::init(argc, argv, "trikh_grid_exec");
    // create node handle to have communication with other nodes.
    ros::NodeHandle n;
    // ros rate/clock of comunication.
    ros::Rate loop_rate(HZ_ROS);


    while(ros::ok())
    {

        
        if(next_node_rsp.response.success)
        {
            // should we override with a specific action?
            if(robot_reactive.override())
            {
                // lets figure what kind of action to perform.
                geometry_msgs::Twist reaction_msg;
                switch(robot_reactive.override())
                {
                    case 0b100:
                        // robot has bumped against something;
                        return(2);
                    case 0b010:
                        // obstacle on the left.
                        
                        reaction_msg.angular.z = double(-1.0);
                        reaction_msg.linear.x = float(0);
                        motorDriver(motor_pub, reaction_msg);
                    case 0b001:
                        // obstacle on the right.
                        reaction_msg.angular.z = double(1.0);
                        reaction_msg.linear.x = float(0);
                        motorDriver(motor_pub, reaction_msg);
                    case 0b011:
                        // obstacle on both sides.
                        reaction_msg.angular.z = double(1.0);
                        reaction_msg.linear.x = float(0);
                        motorDriver(motor_pub, reaction_msg);
                    
                }
            }
        }
}