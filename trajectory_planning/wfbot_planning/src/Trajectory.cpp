#include "ros/ros.h"
#include "wfbot/Trajectory.h"
#include <math.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
namespace Trajectory_ns
{
    void Tclass::Trajectory(double xq, double yq, double x, double y, double z, double ox, double oy, double oz, double ow)
    {

        ros::AsyncSpinner spinner(5);
        spinner.start();

        moveit::planning_interface::MoveGroupInterface arm("arm");

        //获取终端link的名称
        std::string end_effector_link = arm.getEndEffectorLink();

        //设置目标位置所使用的参考坐标系
        std::string reference_frame = "world";
        arm.setPoseReferenceFrame(reference_frame);

        //当运动规划失败后，允许重新规划
        arm.allowReplanning(true);

        //设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.setGoalPositionTolerance(0.001);
        arm.setGoalOrientationTolerance(0.01);

        //设置允许的最大速度和加速度
        arm.setMaxAccelerationScalingFactor(0.8);
        arm.setMaxVelocityScalingFactor(0.8);

        // 控制机械臂先回到初始化位置
        arm.setNamedTarget("home");
        arm.move();
        sleep(1);

        // 设置机器人终端的目标位置
        geometry_msgs::Pose target_pose;
        target_pose.orientation.x = -0.10844113908199787;
        target_pose.orientation.y = 0.10843693378992976;
        target_pose.orientation.z = 0.6987783105412861;
        target_pose.orientation.w = 0.6987065360091482;

        target_pose.position.x = xq;
        //target_pose.position.x = 0.2940212599094821;
        target_pose.position.y = 0.1;
        target_pose.position.z = 0.15;

        arm.setPoseTarget(target_pose);
        arm.move();

        std::vector<geometry_msgs::Pose> waypoints;

        //将初始位姿加入路点列表
        waypoints.push_back(target_pose);

        double centerA = target_pose.position.y;
        double centerB = target_pose.position.z;
        double r2 = 0.01 - (xq - 0.3) * (xq - 0.3);
        double z2 = r2 - (yq - 0.1) * (yq - 0.1);
        double z1 = -1 * sqrt(z2);
        double y1 = yq - 0.1;
        double th2 = atan2((-1 * z1), (-1 * y1));
        int count = int(314 - th2 / 0.01);
        double radius = sqrt(r2);
        double Y = 0.3142;
        //ROS_INFO("centerA:%.10f,centerB%.10f,r2:%.10f,z2:%.10f,z1:%.10f,y1:%.10f,th2:%.10f,count:%d,radius:%.10f",centerA,centerB,r2,z2,z1,y1,th2,count,radius);
        for (double th = 0.0; th < 3.14; th = th + 0.01)
        {

            target_pose.position.y = centerA - radius * cos(th);
            target_pose.position.z = centerB - radius * sin(th);
            // if (target_pose.position.y >= yq)
            // {
            //     target_pose.orientation.x += 0.108373586 / count;
            //     target_pose.orientation.y -= 0.108352798 / count;
            //     target_pose.orientation.z += 0.008398308 / count;
            //     double m = target_pose.orientation.x*target_pose.orientation.x+target_pose.orientation.y*target_pose.orientation.y+target_pose.orientation.z*target_pose.orientation.z;
            //     target_pose.orientation.w =sqrt(1-m);
            // }
            if (target_pose.position.y >= yq)
            {
                Y -= (0.314 / count);
                target_pose.orientation.x = -1 * sin(Y / 2) * 0.707;
                target_pose.orientation.y = sin(Y / 2) * 0.707;
                double m = target_pose.orientation.x*target_pose.orientation.x+target_pose.orientation.y*target_pose.orientation.y+target_pose.orientation.z*target_pose.orientation.z;
               target_pose.orientation.w =sqrt(1-m);
                // ROS_INFO("th:%.10f,ox:%.10f,oy:%.10f,oz:%.10f,ow:%.10f",th,target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w);
            }
            waypoints.push_back(target_pose);
        }
        target_pose.orientation.x = ox;
        target_pose.orientation.y = oy;
        target_pose.orientation.z = oz;
        target_pose.orientation.w = ow;

        target_pose.position.x = x;
        target_pose.position.y = y;
        target_pose.position.z = z;
        waypoints.push_back(target_pose);

        //笛卡尔空间下的路径规划
        moveit_msgs::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = 0.0;
        int maxtries = 100; //最大尝试规划次数
        int attempts = 0;   //已经尝试规划次数

        while (fraction < 1.0 && attempts < maxtries)
        {
            fraction = arm.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            attempts++;

            if (attempts % 10 == 0)
                ROS_INFO("Still trying after %d attempts...", attempts);
        }

        if (fraction == 1)
        {
            ROS_INFO("Path computed successfully. Moving the arm.");

            // 生成机械臂的运动规划数据
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;

            // 执行运动
            arm.execute(plan);
            sleep(1);
        }
        else
        {
            ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
        }

        //控制机械臂先回到初始化位置
        arm.setNamedTarget("home");
        arm.move();
        sleep(1);

        ros::shutdown();
    }
}