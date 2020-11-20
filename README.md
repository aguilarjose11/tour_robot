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
if next_node:
  if no reactions:
    move to next_node
  else:
    react to the specific reaction
```
- next_node is the response from a server from node *trikh_planner* called *next_node*
- *reactions* will specifically be the situation where the robot is about to bump against an obstacle. There exists 3 ways of reaction:
 - Turn away.
  - There is a way to get out
 - Fly away.
  - There is a flat wall in front of us.
 - Stop.
  - The robot has bumped against something and must stop.



#### Obstacle-detection:
For the creating of the world, the robot must be able to detect obstacles before encountering them by the bumper.
For this is that we have decided to make use of obstacle-detection software such as OpenCV to detect obstacles
and categorize them as needed.