#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

#include "std_srvs/SetBool.h"
#include "tour_robot/grid_deltaAction.h"

#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "kobuki_msgs/BumperEvent.h"

#include <math.h>
#include <vector>
#include <stdlib.h>


#define DEBUG true
#define HZ_ROS 10
#define OBSTCL_OFFST 0.2f
#define OFFSET_TBII double(0.3)
#define ANGLE_TOLERANCE float(5)



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

/* Robot Reactive member definitions. */


/*
    Trikh Grid Executive Action

    The Grid Executive action class will perform the main activities 
    required to reach the goal that is specified by the goal message.

    The main way this class perform the former mentioned goals is by
    1. checking that the goal given is valid, 
    2. moving towards the goal in a straight line.
    3. while moving (performing towards goal) check that no obstacles
    are near the robot. if so, reactions are triggered.
    4. if the robot encounters a flat surface where the robot has to turn
    around, we asume that the robot has failed to reach its destination
    and the goal is set to aborted.
    5. if the robot takes longer than 20 seconds, the goal is considered
    unreachable and the robot is set to aborted state.
    6. if the robot is within the goal zone, the goal is considered 
    accomplished and the robot is set to success.

    As this happens, the robot will not send any feedback, but will 
    allow for the main node clients to cancel it.
*/
class TrikhGridExec
{
    private:
        // Obstacle detection call back function
        void obstacleCB(const sensor_msgs::LaserScan msg);
        // Used by the obstacle detection function above.
        int getNextObstacle(sensor_msgs::LaserScan msg);
        // Converts Quartenion units to degrees.
        double getFacedAngle(double z, double w);
        void bumpCheck(const kobuki_msgs::BumperEvent msg);
        // Obstacle types
        enum obstacle_type 
        { 
            NO_OBST, LEFT_OBST, RIGHT_OBST, BIDIR_OBST, HIT_ABORT
        };
        // Obstacle situation. Used by obstacleCB.
        obstacle_type obstacle_flag;
        // Directions of travel
        enum direction_travel
        {
            HALT=0, NORTH=1, SOUTH=-1, EAST=-2, WEST=2, EMPTY=(USHRT_MAX-1)
        };
        direction_travel goal_direction;
        // variables used for obstacle detection.
        unsigned int angle_loc, ranges_len;
        int  goal_x, goal_y;
        bool continual_flag, bump_flag;
        float angle_increment, min_val, angle_rad, total_area_vision, 
        goal_angle;
        const float ANGLE_ERROR_RANGE;
    
    public:
        // Constructor
        TrikhGridExec(std::string name);
        // Destructor
        ~TrikhGridExec(void);
        // Goal call back function
        void goalCB(const tour_robot::grid_deltaGoalConstPtr &goal);
        // Preemption call back function
        void preemptCB();
        // Main action call back function
        void travelCB(nav_msgs::Odometry msg);

    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<tour_robot::grid_deltaAction> as_;
        std::string action_name_;
        tour_robot::grid_deltaFeedback feedback_; // unused
        tour_robot::grid_deltaResult result_;
        ros::Subscriber scan_sub, odom_sub, bump_sub;
        ros::Publisher trvl_pub;
};


/*-------------------------------------------------*/
/* ! TrikhGridExec Member Function Declarations. ! */
/*-------------------------------------------------*/

/* constructor */
/* 
Takes charge of registering the goal and preempt callbacks.
Creates a subscriber for using the obstacle detection
callback.
Starts the action.
*/
TrikhGridExec::TrikhGridExec(std::string name) : 
        as_(nh_, name, boost::bind(&TrikhGridExec::goalCB, this, _1),false),
        action_name_(name),
        ANGLE_ERROR_RANGE(ANGLE_TOLERANCE)
{
    this->goal_direction = EMPTY;
    // Register the preempt callbacks.
    as_.registerPreemptCallback(boost::bind(&TrikhGridExec::preemptCB, this));

    // Create a subscriber for the obstacle detection.
    // Asume obstacleCB is running now!
    scan_sub = nh_.subscribe("/scan", 1000, &TrikhGridExec::obstacleCB, this);
    odom_sub = nh_.subscribe("/odom", 1000, &TrikhGridExec::travelCB, this);
    bump_sub = nh_.subscribe("/mobile_base/events/bumper", 1000, &TrikhGridExec::bumpCheck, this);

    // Create publishers needed for class
    trvl_pub = nh_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velociry", 1000);

    // Start the action
    as_.start();
}

/* Destructor */
/*
Well, the destructor...
*/
TrikhGridExec::~TrikhGridExec(void)
{
    // Such Nothingness...
}

// Returns wether we have bumped against something.
void TrikhGridExec::bumpCheck(const kobuki_msgs::BumperEvent msg)
{
    // If we bump into something, call command.
    this->bump_flag = (msg.state)?(true):(false);
    ROS_INFO("bump state: %d", msg.state);
}


/* goalCB */
/*
Takes care of accepting or rejecting the goal.
The reasons for the rejection of the goal are:
1. The goal requires travel on both axes.
2. The goal requires travel greater than 1m.
if we don't reject, we accept the goal.
Once we accept the goal, we move into the 
traveling portion. at this point, the function
calls for travelCB and returns.
*/
void TrikhGridExec::goalCB(const tour_robot::grid_deltaGoalConstPtr &goal)
{
    // check that we are only moving in a single dirrection
    if(goal->delta_x && goal->delta_y)
    {
        // no going in a 2d direction!
        tour_robot::grid_deltaResult result_;
        result_.success = false;
        as_.setAborted(result_);
        return;
    }
    else
    {
        // we accept!
        // get robot's starting location
        ros::NodeHandle m;
        boost::shared_ptr<nav_msgs::Odometry const> sharedEdge;
        nav_msgs::Odometry edge;
        sharedEdge = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", m);
        if(sharedEdge != NULL)
        {
            edge = *sharedEdge;
        }

        this->goal_x = edge.pose.pose.position.x + goal->delta_x;
        this->goal_y = edge.pose.pose.position.y + goal->delta_y;

        as_.acceptNewGoal();
        if(goal->delta_x > 0)
        {
            this->goal_direction = EAST;
            this->goal_angle = 180;
        }
        else if(goal->delta_x < 0)
        {
            this->goal_direction = WEST;
            this->goal_angle = 360;
        }

        if(goal->delta_y > 0)
        {
            this->goal_direction = NORTH;
            this->goal_angle = 270;
        }
        else if(goal->delta_y > 0)
        {
            this->goal_direction = SOUTH;
            this->goal_angle = 90;
        }
        // pay attention to possible bugs
        // where this is called to infinity
        // when we arrive to goal, but goal
        // is not being registered!
        // symptoms would be timeouts!
    }
    // at this point we should have traveled
    return;
}

/* PreemptCB */
/*
In the case that the node requires the action 
to be preempted, the function simply sets the
action state as such. 
Really, not much going on other than that...
*/
void TrikhGridExec::preemptCB()
{
    ROS_INFO("Travel action is being preempted.\n");
    as_.setPreempted();
    return;
}

/* getNextObstacle */
/*
Functions in a similar way to the function used
in project 3.
This function looks for obstacles in the field
of view and returns the location of its closest
point to the obstacle.

if no obstacles were found, -1 is returned.
! Project 3 Documentation !
"As data is analysed left to right, the function 
looks at the change in distance of the obstacles
Once the function detects an obstacle closer than 
1 foot, it keeps scanning until the distance starts
increasing: which means that we reached the closest 
point of the obstacle, so we can assume that
we have detected a whole obstacle."
*/
int TrikhGridExec::getNextObstacle(sensor_msgs::LaserScan msg)
{

    // only returns minima within 1 foot.
    bool obst_flag = false; 
    while(this->angle_loc < this->ranges_len && __builtin_isnan(msg.ranges.at(this->angle_loc))) // check that we arent already in a minima from a previous obstacle
    {   
        (this->angle_loc)++;
    } // go until we either get to the end of the vector or get outside the 1 ft distance
    while(this->angle_loc < this->ranges_len && this->continual_flag && msg.ranges.at(this->angle_loc) < float(0.3048 + OFFSET_TBII))
    {   
        (this->angle_loc)++; // we have already found an obstacle. probably this is it still that one.
    }
    int loc_obst = -1;
    for(; this->angle_loc < ranges_len; (this->angle_loc) += 1) // move until the end of the vector. may not reach it tho.
    {
        
        if(msg.ranges.at(this->angle_loc) < float(0.3048 + OFFSET_TBII) && !obst_flag) // if we have found a close obstacle
        {
            obst_flag = true; // start checking until we find something not as a minimum
            loc_obst =  this->angle_loc;
            continue;
        }
        if(msg.ranges.at(this->angle_loc) < float(0.3048 + OFFSET_TBII) && obst_flag) // only smaller numbers from now on.
        {
            if(msg.ranges.at(loc_obst) >= msg.ranges.at(this->angle_loc)) // we find a new smaller value
            {   
                loc_obst = this->angle_loc; // set new obstacle's location
                continue;
            }
            else
            {   
                this->continual_flag = true; // This lets us know that we have encounter an obstacle, so more could come. 
                break; // new small value is not smaller than previous. we passed the pinnacle. break.         
            }
        }
    }
    return loc_obst;


    return(-1);
}

/* obstacleCB */
/*
The main call back function used to update the 
member flag `obstacle_flag`.
This call back function recieves a message from
the topic its listening (/scan) and does the 
following:
1. Retrieves all of the locatons of obstacles
using the getNextObstacle function
2. Checks to see if the obstacles fall in the
left or right field of view of the robot.
based on this, updates the flag variables.
if there is none, then set them to none.
*/
void TrikhGridExec::obstacleCB(sensor_msgs::LaserScan msg)
{
    // First and foremost check we haven't crashed
    if(this->bump_flag)
    {
        // Robot crashed against something
        this->bump_flag = HIT_ABORT;
        return;
    }

    // Vector to contain the locations(ray beam number) of every obstacle
    std::vector<double> obstacle_locations;
    // get the increment per location
    this->angle_increment = msg.angle_increment;
    // reset the initial scanning variable for the data back to the begining (leftmost beam)
    this->angle_loc = 0;
    // reset the min_val to leftmost beam (similar to a minimum algorithm)
    this->min_val = msg.ranges.at(this->angle_loc);
    // initializes the initial angle in radians
    this->angle_rad = msg.angle_min + (this->angle_loc * this->angle_increment);
    // Gets the total range of vision
    this->total_area_vision = abs(msg.angle_min) + abs(msg.angle_max);
    // gets the number of beams that were shot
    this->ranges_len = msg.ranges.size();
    // resets the continual flag (turn on when we have found an obstacle and more could come.)
    this->continual_flag = false;
    while(true) // get all posible obstacles.
    {
        int val = this->getNextObstacle(msg);
        if(val == -1)
        {
            break;// getNextObstacle returns -1 when no new obstacles were found
        }
        obstacle_locations.push_back((double(val) * this->angle_increment) + msg.angle_min);
    }
    // no obstacles?
    if(obstacle_locations.size() == 0)
    {
        this->obstacle_flag = NO_OBST;
        return;
    }
    // determine the direction of the obstacles
    bool right_flag, left_flag;
    for(int i = 0; i < obstacle_locations.size(); i++) // for every obstacle that was detected...
    {
        if(obstacle_locations.at(i) < 0) // mainly on the left.
        {
            right_flag = true;
        }
        else // mainly on the right.
        {
            left_flag = true;
        }

    }
    if(right_flag && left_flag)
    {
        // the obstacle is bidirrectional!
        this->obstacle_flag = BIDIR_OBST;
        return;
    }
    else if(right_flag)
    {
        // obstacle is on the left.
        this->obstacle_flag = LEFT_OBST;
        return;
    }
    else if(right_flag)
    {
        // only direction left is right
        this->obstacle_flag = RIGHT_OBST;
    }
    else
    {
        // possible bug
        ROS_WARN("Possible bug: there is obstacles but no direction?\n");
        return;
    }
    return;
}

/* getFacedAngle */
/*
Converts quartenion units into degrees.
Follows the formula: (arccos(w)*2)*180/PI = acos(w) * 360 / PI
X and Y are omitted because of the physical limitations of the
robot.
Taken from project 1-3.
*/
double TrikhGridExec::getFacedAngle(double z, double w)
{

    if(z < 0)
    {
        return (acos(-w) * 360.0) / M_PI;
    }
    else
    {
        return (acos(w) * 360.0) / M_PI;
    }
}


/* travelCB */
/*
The action's callback function used for actual
traveling. The function introduces a reactive
system into the robot architecture by reveling
against instructions if the robot is about to
have contact with an obstacle.

This function checks before sending an executive
instruction to the robot's movement topic wether
the robot is in close proximity or in contact
with an obstacle. when that happens, the robot
either turns away, and tries to find its way out 
of the obstacle, or simply stops and "gives up"

If the robot reaches its goal, the action is set
to success and returns a true flag.
*/
void TrikhGridExec::travelCB(nav_msgs::Odometry msg)
{
    geometry_msgs::Twist msg_;
    if(!as_.isActive())
    {
        ROS_WARN("Action not active. possible bug!");
        return;
    }
    // get current location.
    int curr_x, curr_y;
    curr_x = msg.pose.pose.position.x;
    curr_y = msg.pose.pose.position.y;
    // Check wether goal has been reached.
    if(curr_x == goal_x && curr_y == goal_y)
    {
        // goal reached
        tour_robot::grid_deltaResult result_;
        result_.success = true;
        as_.setSucceeded(result_);
    }

    // get current faced direction.
    double curr_angle;
    curr_angle = getFacedAngle(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
    // find out if we are facing the right dirrection
    if(abs(curr_angle - this->goal_angle) > ANGLE_ERROR_RANGE)
    {
        // robot faces wrong direction.
        // Calculate the direction of turn.
        float diff_angle = curr_angle + 180;
        float offset = 0;
        if(diff_angle > 360)
        {
            offset = abs(360 - diff_angle); // 0 -> offset
            if(this->goal_angle >= curr_angle || this->goal_angle <= offset)
            {
                // in positive area
                msg_.angular.z = 1;
            }
            else
            {
                // in negative area
                msg_.angular.z = -1;
            }
        }
        else
        {
            if(this->goal_angle >= curr_angle && this->goal_angle <= diff_angle)
            {
                // in positive area
                msg_.angular.z = 1;
            }
            else
            {
                // in negative area
                msg_.angular.z = -1;
            }

        }

    }
    else
    {
        // Robot faces the correct direction.
        msg_.linear.x = double(1.0); // move forward
    }
    /* Reactive section */
    // These actions override the message that will be sent
    if(this->obstacle_flag != NO_OBST)
    {
        // Robot encountered an obstacle.
        // react!
        msg_.linear.x = 0;
        msg_.angular.z = 0;
        switch (this->obstacle_flag)
        {
        case LEFT_OBST:
            msg_.angular.z = double(-1);
            break;
        case RIGHT_OBST:
            msg_.angular.z = double(1);
            break;
        case BIDIR_OBST:
        {
            // double obstacle: abort!
            tour_robot::grid_deltaResult result_;
            result_.success = false;
            as_.setAborted(result_);
            break;
        }
        case HIT_ABORT:
        {
            // crashed against something: abort!
            tour_robot::grid_deltaResult result_;
            result_.success = false;
            as_.setAborted(result_);
            break;
        }
        default:
        ROS_WARN("default reaction breached!");
            break;
        }
    }

    // Make desired move.
    trvl_pub.publish(msg_);
    return;
}

int main(int argc, char **argv)
{
    // initialize the executive node.
    ros::init(argc, argv, "trikh_grid_exec");
    // spin the node.
    ros::spin();
}
