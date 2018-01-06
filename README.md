# CarND-Path-Planning-Project

The project consists of finding optimal path for driving on 3-lane highway. The goal is to stay as close to speed limit as possible without exceeding it. Constraints are 
1. Do not exceeding maximum acceleration of 10 m/s^2 and jerk of 10 m/s^3
2. Avoid any collisions.

Provided input:
1. Localization information
   <p>Ego car's location in Cartesian and Frenet coordinates and velocity are provided
2. Sensor fusion
   <p>All cars in environment and their location and velocity is provided

Output:
1. Set of trajectory points in front of car 

The actuator in simulator would traverse those points perfectly with each point taking 0.02 seconds.

### Trajectory generation
Trajectory is generated using spline library. At every cycle 5 points are selected, two of which are last  points
of previous trajectory and 3 points are extrapolation of those points into target lane (given by d in Frenet coordinates) 30 meters apart. These 5 points generate a spline which passes through all these points.
After that 50 points on that spline are selected in a way that target speed can be achieved.

### Lane change behavior
Lane change behavior is modeled using a Finite State Machine (FSM) transitions.
5 states are chosen.
 1.   KEEP_LANE
 2.   CHANGE_LEFT
 3.   CHANGE_RIGHT
 4.   PREPARE_LEFT_CHANGE
 5.   PREPARE_RIGHT_CHANGE


These states are defined below
#### KEEP_LANE 
Keep lane pretains to state where car stays in current lane. This is done when
* There is no car in front of ego car. Try to got at maximum speed
* There is car in front of ego car but adjacent lanes have slower traffic. Try to go at speed of car in front
#### CHANGE_LEFT/CHANGE_RIGHT 
* There is a slower car in front in current lane and moving to left/right lane is safe.
#### PREPARE_LEFT/RIGHT_CHANGE
* There is a slower car in font in current lane but moving to adjacent lane is not safe. Go at speed of front car
   and wait for space to open up in adjacent lane. In case car can chose between both left and right lanes, left is always 
   preferred assuming left lanes are faster lanes.
   
Safe distance is defined as
* No car 30 meters ahead in front lane.    
* No car between 20 meters ahead and 10 meters behind in adjacent lanes

##### State Transitions 
1. KEEP_LANE can transit into CHANGE_LEFT/RIGHT and PREPARE_LEFT/RIGHT_CHANGE.
2. PREPARE_LEFT/RIGHT_CHANGE can transit back to KEEP_LANE or CHANGE_LEFT/RIGHT
3. CHANGE_LEFT/RIGHT will transit to KEEP_LANE
4. All States can transit to same states.

At every cycle next state transitions are calculated again.

## Implementation details

A class CarStatus is defined to hold information about the current status of car
1. Current Car State
2. Current Speed
3. Current Lane

NextAction() function implements the automata for FSM. It takes information about sensor fusion and current car information and calculates 
1. Next Car State
2. Target Speed
3. Target Lane

It makes use of checkLaneSafe() function

In main code a single CarStatus object is used to track cars status. At start it uses NextAction to predict the next behavorial step.
Afterwards trajectory is created to achieve that behavior using splines as explained earlier.
   
