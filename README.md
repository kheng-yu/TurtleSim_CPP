This is my first script to practice C++ with ROS, using turtlesim

The purpose of the script is to to control turtlesim through C++, 
implementing the following functions
1) Specify movement and direction to turtle
2) Specify a specific spot (goal) the turtle should go to

As of V1, it is done by
1) move() function which publishes commands to /turtle1/cmd_vel using geometry_msg/Twist
2) Using time based approach for easier implementation, however results in an error of about 0.3m
3) Subscribing to turtlesim/Pose to obtain current pose (location and orientation) of the turtle, this allows one to determine the distance and angle to turn, and can use move() to get to the goal

TODO:
1) Create a class object for turtle, as practice and also for readability
2) Making it Linear Time Independent (LTI) by removing all time-based implementation, improving accuracy
