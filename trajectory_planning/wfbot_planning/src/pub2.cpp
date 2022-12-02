#include"ros/ros.h"
#include"std_msgs/String.h"
#include"sstream"
#include"fwbot/qucan.h"
#include"fwbot/songcan.h"

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"camera2");
    ros::NodeHandle nh2;
    ros::Publisher pub2 = nh2.advertise<fwbot::songcan>("songcan",10);
    fwbot::songcan pose;
    pose.x = 0.319415430964476;
    pose.y = 0.196223376922975;
    pose.z = 0.292518713145199;
    pose.ox = 0.0000818758132576827;
    pose.oy = 0.000155700897880791;
    pose.oz = 0.589077499176542;
    pose.ow = 0.808076524233622;
    ros::Rate rate(10);
    while(ros::ok())
    {
        pub2.publish(pose);
        rate.sleep();
        ros::spinOnce();
    }
}