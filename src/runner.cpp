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
    const double KP_DIST = 5;
    const double KP_ANGLE = 5;
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
    // Get Linear Velocity of turtle
    double getLinVel()
    {
        return linvel;
    }
    // Get Angular Velocity of turtle
    double getAngVel()
    {
        return angvel;
    }

    // Move the robot for a certain distance at a certain speed
    // topic: turtle1/cmd_vel
    // msg type: geometry_msgs/Twist
    // args: linear angular: [x,y,z] [x,y,z]
    void move(double dist, double speed, bool isForward, double angle)
    {
        // Update position
        ros::spinOnce();

        // Initialize msg
        geometry_msgs::Twist vel_msg;

        // Time based implementation
        double initialtime, traveltime, finaltime;

        // Define Angular Velocity constant
        const double ANG_VEL = 90*M_PI/180;

        // Check if to turn
        if(angle!= 0)
        {
            if(angle  > 0)
            {
                vel_msg.angular.z = ANG_VEL; 
            }
            else
            {
                vel_msg.angular.z = -ANG_VEL; 
            }

            initialtime = ros::Time::now().toSec();
            traveltime = abs(angle)/ANG_VEL;
            finaltime = initialtime + traveltime;

            ROS_INFO("Begin alining direction!");
        
            // Begin turning until specified angle
            velocityPublisher.publish(vel_msg);
            while(ros::Time::now().toSec() <= finaltime)
            {
                velocityPublisher.publish(vel_msg);
                ros::spinOnce();
                // ROS_INFO("this->angle: %lf \n", this->angle);
                // ROS_INFO("angle: %lf \n", angle);
                // ROS_INFO("diff: %lf \n", this->angle - angle);
            }

            vel_msg.angular.z = 0;
            velocityPublisher.publish(vel_msg); // Stop turning

            ROS_INFO("Finished aligning!");
        }

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
            ros::spinOnce();
        }
        // Reached specified distance
        vel_msg.linear.x = 0;
        
        // Publish again
        velocityPublisher.publish(vel_msg);
        
        ROS_INFO("Turtle reached its destination!\n");
    }
    
    // Move the robot to a specified location, moving while rotating at the same time
    void smoothmove(double dest_x, double dest_y, double error)
    {
        // Initialize msg
        geometry_msgs::Twist vel_msg;

        // Initialize Variables
        double move_x, move_y, move_dist, dest_angle, move_angle;
        
        // Initialize frequenccy
        ros::Rate loop_rate(62);

        // Get initial
        move_x = dest_x - this->x;
        move_y = dest_y - this->y;
        move_dist = sqrt((move_x*move_x) + (move_y*move_y));
        
        // Have not reached
        while(move_dist > error)
        {
            // Set forward velocity
            vel_msg.linear.x = KP_DIST * move_dist;
            if(vel_msg.linear.x > 5)
            {
                vel_msg.linear.x = 5;
            }

            // Compute angle to turn
            dest_angle = std::atan2(move_y,move_x);
            move_angle = dest_angle - this->angle;
            
            // Have to fix for facing left, as it fluctuates between PI and -PI
            // This also lets it turn more efficiently (turns the shorter distance to reach dest_angle)
            while(abs(move_angle) > M_PI)
            {
                if(move_angle > 0) // Should turn right instead
                {
                    move_angle -= 2*M_PI;
                }
                else // Should turn left instead
                {
                    move_angle += 2*M_PI;
                }
            }

            // Scale angle error as velocity to turn (Proportional Controller)
            vel_msg.angular.z = KP_ANGLE * move_angle;
            
            velocityPublisher.publish(vel_msg);

            // Update position
            ros::spinOnce();

            // Get new distance
            move_x = dest_x - this->x;
            move_y = dest_y - this->y;
            move_dist = sqrt((move_x*move_x) + (move_y*move_y));

            loop_rate.sleep();
        }

        // Should reach destination by now
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        velocityPublisher.publish(vel_msg);

        ROS_INFO("x: %lf, y:%lf \n", this->x, this->y);    
    }

    // Update turtle object's pose when receive /turtlesim/Pose/ msgs
    // topic: turtle1/pose
    // msg type: turtlesim/Pose
    // args: [double] x y theta linear_velocity angular_velocity
    void updatePose(const turtlesim::Pose::ConstPtr& msg)
    {
        this->x = msg->x;
        this->y = msg->y;
        this->angle = msg->theta; // in rads
        this->linvel = msg->linear_velocity; // in m/s
        this->angvel = msg->angular_velocity; // in rads

        // std::cout << "Updating pose, "<< "\n" << "x: " << msg->x << "\n";
        // std::cout << "y: " << msg->y << "\n";
        // std::cout << "Current angle: " << this->angle << "\n";
        // // std::cout << "linvel: " << msg->linear_velocity << "\n";
        // // std::cout << "angvel: " << msg->angular_velocity << "\n";
    }
};

/* Function List */

bool instruction(Turtle& turtle);

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
bool instruction(Turtle& turtle)
{
    // Initialize Variables
    int command;
    double dist, speed, angle;
    bool isForward;

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
        std::cout << "How many degrees to the left would you like to turn? ";
        while(!(std::cin >> angle))
        {
            std::cout << "You have entered a wrong input, please specify a number\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "How many degrees to the left would you like to turn? ";
        }

        turtle.move(dist, speed, isForward, angle*M_PI/180);
    }
    
    else if(command == 2)
    {
        // Update Position
        ros::spinOnce();
        
        // Initialize Variables
        double goto_x, goto_y, error;

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
        std::cout << "error threshold = ";
        while(!(std::cin >> error))
        {
            std::cout << "You have entered a wrong input, please specify with numbers only.\n";
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            std::cout << "error = ";
        }

        turtle.smoothmove(goto_x, goto_y, error);
    }

    else // entered random number, exit
    {
        std::cout << "Exiting program...";
        return 1;
    }
}