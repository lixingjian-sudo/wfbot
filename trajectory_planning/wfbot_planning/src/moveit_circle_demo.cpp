#include <math.h>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
// #include "fwbot/T1.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "moveit_cartesian_demo");
    // T1_ns::QuCan my;
    double x = 0.3;
    double y = 0;
    // my.T1(x,y);
    return 0;
}
