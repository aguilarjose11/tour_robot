Final Inteligent Robotics Project: "Tri-kh" -> "Tripode Khryseoi"
=================================================================

The main objective of the project is to create a robot capable of touring a OU facility.
For the achievement of this goal, we will use the ROS robotics framework.

Design:
-------

For our implementation, we have decided to follow a hybrid approach to our robotical architecture.
We will implement a 3T-based system where we have 3 main layers:
1. Deliberative:
 - Our robot will use self-organizing snn's to create a representation of the world, and using
   an algorithm similar to A* the path planning will create dynamic plans.
2. Reactive:
 - Alongside the robotical Executive module, there will be a reactive part to the robot where specific
   events will cause an override of actions and cause specific actions to happen.
   - Bumping against a wall.
   - Getting too close to an obstacle. (turn away or fly away.)
   - stop signal?
3. Coordinating layer:
 - For the coordination of the two above layers, we will follow a "do-until-react" philosophy.
   What this means is that the robot will follow a path plan until there is a reaction. If there
   is a need to update the path plan, the the robot does so. We are expecting the low-energy
   requirements of the SNN path planning layer to help reduce the energy/computational expense.
   This will be a good research point for the future.

Architecture:
-------------

### Nodes:

#### Path-planning:
For the pourpose of path planning, we are researching into the posibility of the use of Spiking Neuronal
Networks as a possible euclidean metric map representation. What we mean by this is using the timed-nature
of neuronal networks to identify best possible paths to a goal based on who triggers first. Similar attempts
have been performed in research/literature before as in _Alamdari[1]_, _bouganis et al._, and _Lobo et al._

#### Executive: trikh_executive
The Executive layer is mainly governed by the coordinating layer. In this layer, the robot will decide what action to take next. The way the robot achieves this is with the following algorithm:
```
if there is next node:
  run to the node
```
- next_node is the response from a server from node *trikh_planner* called *next_node*
- *reactions* will specifically be the situation where the robot is about to bump against an obstacle. There exists 3 ways of reaction:
 - Turn away.
  - There is a way to get out
 - Fly away.
  - There is a flat wall in front of us.
 - Stop.
  - The robot has bumped against something and must stop.
These reactions will be found in the *trikh_grid_exec* node.

To check for the reactions, the node uses a class called RobotReactive.

##### RobotReactive
This class takes care of revising the environment for obstacles using a primitive approach: **If robot is bumping or within _offset_ distance from an obstacle, react.**

###### char override()
The class uses a 3 bit flag called *stimulus* which is a private variable. This function acts as an accesor to the forementioned variable. The strcture of the flags is the following: 1st bit -> bumper, 2nd bit -> obstacle to the left, 3rd bit -> obstacle to the right.



#### Obstacle-detection:
For the creating of the world, the robot must be able to detect obstacles before encountering them by the bumper.
For this is that we have decided to make use of obstacle-detection software such as OpenCV to detect obstacles
and categorize them as needed.


Development:
============

*November 23 update*

## Trikh_grid_exec:
1. [ ] Update to use *grid_deltaAction* action from SetBool.
2. [ ] Reshape body of main to work as an action
3. [ ] Implement reactive actions.
4. [ ] Implement Result-trigging code
5. [ ] Implement Feedback code. (?)

## Trikh_user_interface:
1. [ ] Create service code body
2. [ ] create comunication chanel with trikh_user_teleop called new_user_goal which is a service that will update a data structure containing the current goal. Designed to override any current goals. set a global boolean to tell wether we have a new goal (is_new_goal)
3. [ ] If the service draft_new_plan is called, return the is_new_goal boolean and set it to false afterwards.

## Trikh_user_teleop:
1. [ ] create regular node body of code.
2. [ ] print input interface message explaining how the interface will accept input (x, y)
3. [ ] Start ros loop where we check for any input from the user
4. [ ] when we get some we check that it conforms the input standards.
5. [ ] If input is acceptable, send the new input coordinates by creating a new message *CartesianOdom*
6. [ ] via service, send the new coordinates with user_goal to trikh_user_interface.
7. [ ] Set a way for the user to exit the loop (entering ctrl + c for example)

## trikh_odom
1. [ ] Set up as a regular simple service
2. [ ] Create body code for a service called current_location
3. [ ] call back function does following:
  1. [ ] Get /Odom topic's single message
  2. [ ] convert the odom data into message CartesianOdom by rounding the odometry data to whole numbers.
  3. [ ] Send the data back as the response.
  
 ## trikh_planner
 Research!
