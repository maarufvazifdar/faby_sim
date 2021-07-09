/**
 * @author Mohammed Maaruf Vazifdar
 */

#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<nav_goals/nav_goals.h>

int main(int argc, char **argv)
{   
    while(true)
    {
        // Create a node 'navigation_goals' to run this code.
        ros::init(argc, argv, "navigation_goals");
        ros::NodeHandle nh;

        // Create an object of class Send_goal.
        Send_goal s1;

        // create a subscriber to the "/odom" topic so we can get the odometry message.
        ros::Subscriber sub = nh.subscribe("odom", 1000, &Send_goal::PoseCallback,&s1);

        // Calls function select_goal of class Send_goal which asks for input from user.
        s1.select_goal();
    }
    return 0;
}