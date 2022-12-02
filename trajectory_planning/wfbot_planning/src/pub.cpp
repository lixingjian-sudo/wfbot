#include"ros/ros.h"
#include"std_msgs/String.h"
#include"sstream"
#include"fwbot/qucan.h"
#include"fwbot/songcan.h"

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"camera");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<fwbot::qucan>("qucan",10);
    fwbot::qucan position;
    position.x = 0.3;
    position.y = 0.1;
    ros::Rate rate(10);
    while(ros::ok())
    {
        pub.publish(position);
        rate.sleep();
        ros::spinOnce();
    }
}