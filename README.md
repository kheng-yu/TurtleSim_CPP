This is my first script to practice C++ with ROS (Melodic), using turtlesim

The purpose of the script is to to control turtlesim through C++, 
implementing the following functions
1) Specify movement and direction to turtle
2) Specify a specific spot (goal) the turtle should go to

To use:
1) Put entire content inside <catkin_workspace>/src/turtlesim_runner
2) Run catkin_make on <catkin_workspace>
3) To run the script, you may need to first do source devel/setup.bash
4) Run roscore, turtlesim_node then script, which is rosrun turtlesim_runner runner
5) Follow script prompts

As of V1.2, the code is
1) A turtle class which holds the turtle's current pose (coordinates & orientation)
2) move() function which publishes commands to /turtle1/cmd_vel using geometry_msg/Twist
3) Subscribing to turtlesim/Pose to obtain current pose (location and orientation) of the turtle, this allows one to determine the distance and angle to turn, and can use move() to get to the goal
4) Almost Linear Time-Invariant (LTI), smoothmove() allows it to achieve an accuracy up to the error threshold from user.

TODO:
1) Make move() LTI as well.

------------------------------------------------------------------
History & Changelog:
V1.2
1) Made new smoothmove() for travelling to specific spot function
2) smoothmove() is LTI, can achieve accuracy up to error threshold from user
3) Implemented a Proportional controller for smoothmove()

V1.1
1) Turtle Class made
2) Code block comments in runner.cpp removed

V1
1) move() function which publishes commands to /turtle1/cmd_vel using geometry_msg/Twist
2) Using time based approach for easier implementation, however results in an error of about 0.3m
3) Subscribing to turtlesim/Pose to obtain current pose (location and orientation) of the turtle, this allows one to determine the distance and angle to turn, and can use move() to get to the goal
