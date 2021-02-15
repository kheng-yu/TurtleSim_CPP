This is my first script to practice C++ with ROS (Melodic), using turtlesim

The purpose of the script is to to control turtlesim through C++, 
implementing the following functions
1) Specify movement and direction to turtle
2) Specify a specific spot (goal) the turtle should go to

To use:
1) Put entire content inside <catkin_workspace>/src/turtlesim_runner
2) Run catkin_make on <catkin_workspace>
3) Run roscore, turtlesim_node and script
4) Follow script prompts

As of V1.1, it is done by 
1) A turtle class which holds the turtle's current pose (coordinates & orientation)
1) move() function which publishes commands to /turtle1/cmd_vel using geometry_msg/Twist
2) Using time based approach for easier implementation, however results in an error of about 0.3m
3) Subscribing to turtlesim/Pose to obtain current pose (location and orientation) of the turtle, this allows one to determine the distance and angle to turn, and can use move() to get to the goal

TODO:
1) Making it Linear Time Independent (LTI) by removing all time-based implementation
2) Online tutorial for Python (http://wiki.ros.org/turtlesim/Tutorials/Go%20to%20Goal) uses a PID controller to reduce error, a similar approach will be taken for this project

------------------------------------------------------------------
History & Changelog:
V1.1
1) Turtle Class made
2) Code block comments in runner.cpp removed

V1
1) move() function which publishes commands to /turtle1/cmd_vel using geometry_msg/Twist
2) Using time based approach for easier implementation, however results in an error of about 0.3m
3) Subscribing to turtlesim/Pose to obtain current pose (location and orientation) of the turtle, this allows one to determine the distance and angle to turn, and can use move() to get to the goal
