/* 
This is my first script to practice C++ with ROS, using turtlesim

The purpose of the script is to to control turtlesim through C++, 
implementing the following functions
1) Specify movement and direction to turtle
2) Specify a specific spot (goal) the turtle should go to
*/

#include "ros/ros.h" // Must include for all ROS C++
#include "geometry_msgs/Twist.h" // To control linear and angular velocity of turtle
#include "turtlesim/Pose.h" // To obtain coodinates and orientation of turtle
#include <sstream>

// Turtle Class
class Turtle{
    private:
    // Variables for Current Pose
    double x;
    double y;
    double angle;
    double linvel;
    double angvel;
    // ROS
    ros::NodeHandle n; // Create its specific node handler
    ros::Publisher velocityPublisher;
    ros::Subscriber poseSubscriber;

    public:
    // Constructor
    Turtle()
    {
        this->velocityPublisher = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);
        this->poseSubscriber = n.subscribe("/turtle1/pose", 10, &Turtle::updatePose, this);
        ros::Rate loop_rate(1);
    }

    /* Turtle Class Functions */
    // Get X of turtle
    double getX()
    {
        return x;
    }
    // Get Y of turtle
    double getY()
    {
        return y;
    }
    // Get orientation of turtle in radians
    double getAngle()
    {
        return angle;
    }

    // Move the robot for a certain distance at a certain speed
    // topic: turtle1/cmd_vel
    // msg type: geometry_msgs/Twist
    // args: linear angular: [x,y,z] [x,y,z]
    void move(double dist, double speed, bool isForward, double angle, bool toTurn)
    {
        // Initialize msg
        geometry_msgs::Twist vel_msg;

        // Define Angular Velocity constant
        const double ANG_VEL = 45*M_PI/180; // 45 degree/s rotation

        // Turn first
        if(toTurn)
        {
            if(angle  >= 0)
            {
                vel_msg.angular.z = ANG_VEL; 
            }
            else
            {
                vel_msg.angular.z = -ANG_VEL; 
            }
        }
        double initialtime = ros::Time::now().toSec();
        double traveltime = abs(angle)/ANG_VEL;
        double finaltime = initialtime + traveltime;
        ROS_INFO("Begin alining direction!");
        
        // Begin turning until specified angle
        velocityPublisher.publish(vel_msg);
        while(ros::Time::now().toSec() <= finaltime)
        {
            velocityPublisher.publish(vel_msg);
        }

        vel_msg.angular.z = 0;
        velocityPublisher.publish(vel_msg); // Stop turning

        ROS_INFO("Finished aligning!");

        // Set forward or backwards movement
        if(isForward)
        {
            vel_msg.linear.x = abs(speed);
        }
        else
        {
            vel_msg.linear.x = -abs(speed);
        }

        //Publish Move
        velocityPublisher.publish(vel_msg);

        ROS_INFO("Turtle is moving!\n");

        // Stop when reaching the distance specified
        initialtime = ros::Time::now().toSec();
        traveltime = dist/speed;
        finaltime = initialtime + traveltime;
        while (ros::Time::now().toSec() <= finaltime)
        {
            velocityPublisher.publish(vel_msg);
        }
        // Reached specified distance
        vel_msg.linear.x = 0;
        
        // Publish again
        velocityPublisher.publish(vel_msg);
        
        ROS_INFO("Turtle reached its destination!\n");
    }
    
    // Update turtle object's pose when receive /turtlesim/Pose/ msgs
    // topic: turtle1/pose
    // msg type: turtlesim/Pose
    // args: [double] x y theta linear_velocity angular_velocity
    void updatePose(const turtlesim::Pose::ConstPtr& msg)
    {
        x = msg->x;
        y = msg->y;
        angle = msg->theta; // in rads
        linvel = msg->linear_velocity; // in m/s
        angvel = msg->angular_velocity; // in rads
    }
};

/* Function List */

bool instruction(Turtle turtle);

int main(int argc, char **argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "robot_runner");

    //Initialize Turtle Object
    Turtle turtle = Turtle();

    while(ros::ok()) // while ros is running, repeat this 
    {
        if(instruction(turtle))
        {
            // exit is run, exit program
            return 1;
        }

        ros::spinOnce();
    }

    return 0;
}

// Instructions for main()
// Prompts for user input to determine what the passed in the turtle object should do
bool instruction(Turtle turtle)
{
    // Initialize Variables
    int command;
    double dist, speed, angle;
    bool isForward, toTurn;

    // Ask for command
    std::cout << "Hello, this program has two functions. Please enter 1 or 2.\n";
    std::cout << "1) Specify movement and direction to turtle.\n";
    std::cout << "2) Specify location for turtle to travel to.\n";
    std::cout << "Any other number => Exit\n";
    while(!(std::cin >> command))
    {
        std::cout << "You have entered a wrong input, please only specify numbers. ";
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    
    if (command == 1)
    {
        std::cout << "What is the distance? ";
        while(!(std::cin >> dist))
        {
            std::cout << "You have entered a wrong input, please specify distance with numbers only.\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "What is the distance? ";
        }
        std::cout << "What is the speed to travel at? ";
        while(!(std::cin >> speed))
        {
            std::cout << "You have entered a wrong input, please specify speed with numbers only.\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "What is the speed to travel at? ";
        }
        std::cout << "Are you moving forwards? Answer 1 for true or 0 for false. ";
        while(!(std::cin >> isForward))
        {
            std::cout << "You have entered a wrong input, please specify 1 or 0.\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Are you moving forwards? Answer 1 for true or 0 for false. ";
        }
        std::cout << "Would you like to turn before moving? Answer 1 for true or 0 for false. ";
        while(!(std::cin >> toTurn))
        {
            std::cout << "You have entered a wrong input, please specify true or false.\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "Would you like to turn before moving? Answer 1 for true or 0 for false. ";
        }
        if(toTurn)
        {
            std::cout << "How many degrees to the left would you like to turn? ";
            while(!(std::cin >> angle))
            {
                std::cout << "You have entered a wrong input, please specify a number\n";
                std::cin.clear();
                std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
                std::cout << "How many degrees to the left would you like to turn? ";
            }
        }

        turtle.move(dist, speed, isForward, angle*M_PI/180, toTurn);
    }
    
    else if(command == 2)
    {
        double goto_x, goto_y;

        std::cout << "Current pose of turtle is\n";
        std::cout << "x: " << turtle.getX() << "\n";
        std::cout << "y: " << turtle.getY() << "\n";
        std::cout << "angle: " << turtle.getAngle()*180/M_PI << "\n";
        std::cout << "Where would you like the turtle to go to?\n";
        std::cout << "x = ";
        while(!(std::cin >> goto_x))
        {
            std::cout << "You have entered a wrong input, please specify with numbers only.\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "x = ";
        }
        std::cout << "y = ";
        while(!(std::cin >> goto_y))
        {
            std::cout << "You have entered a wrong input, please specify with numbers only.\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "y = ";
        }

        // Got the destination, now orient and move
        double move_x = goto_x - turtle.getX();
        double move_y = goto_y - turtle.getY();
        double move_dist = sqrt((move_x*move_x) + (move_y*move_y));
        double dest_angle = std::atan2(move_y,move_x);
        double move_angle = dest_angle - turtle.getAngle();
        ROS_INFO("Move Angle = %lf", move_angle*180/M_PI);
        
        turtle.move(move_dist, 5, 1, move_angle, 1); // slight errors
    }

    else // entered random number, exit
    {
        std::cout << "Exiting program...";
        return 1;
    }
}