#include "ros/ros.h"
#include "wfbot/qucan.h"
#include "wfbot/songcan.h"
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include "wfbot/Trajectory.h"
double xq;
double yq;
double x;
double y;
double z;
double ox;
double oy;
double oz;
double ow;

void doMsg1(const wfbot::qucan::ConstPtr &position){
     xq = position->x;
     yq = position->y;
     double count;
     count ++;
    // ROS_INFO("%.2f,%.2f",xq,yq);
    // loop_rate.sleep();
}

void doMsg2(const wfbot::songcan::ConstPtr &pose){
     x = pose->x;
     y = pose->y;
     z = pose->z;
     ox = pose->ox;
     oy = pose->oy;
     oz = pose->oz;
     ow = pose->ow;
     double count;
     count ++;
    // ROS_INFO("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",x,y,z,ox,oy,oz,ow);
    // loop_rate.sleep();
}

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"moveit");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(5);
    ros::Subscriber sub1 = nh.subscribe("qucan",10,doMsg1);
    ros::Subscriber sub2 = nh.subscribe("songcan",10,doMsg2);
    spinner.start();
    ros::Rate loop_rate(3);
    loop_rate.sleep();
    spinner.stop();
    Trajectory_ns::Tclass my;
    my.Trajectory(xq,yq,x,y,z,ox,oy,oz,ow);
    return 0;
}
