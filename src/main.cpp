#include <ros/ros.h>
#include <wiringPi.h>

#include "robot_wheel_speeds/DifferentialDriveEncoders.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "robot_wheel_speeds");
    wiringPiSetupSys();

    ros::NodeHandle nodeHandle;

    auto publisher = nodeHandle.advertise<robot_wheel_speeds::WheelVelocities>(
                        "wheel_speeds", 10);

    ros::Rate loop_rate(20);

    // Config
    int velocityUpdateIntervalNs = 2e7;
    ros::Duration velocityUpdateInterval(0, velocityUpdateIntervalNs);
    int leftPinA = 17;
    int leftPinB = 18;
    int rightPinA = 22;
    int rightPinB = 23;
    int ticksPerRevolution = 300;
    int wheelDiameterMm = 60;
    int wheelAxisMm = 70;

    ROS_INFO("Starting wheel_speeds");
    DifferentialDriveEncoders encoders(leftPinA, leftPinB, rightPinA, rightPinB,
                                       ticksPerRevolution, wheelDiameterMm, wheelAxisMm,
                                       velocityUpdateInterval);

    encoders.Start();
    ROS_INFO("Encoders started");
    int seq = 1;
    while(ros::ok())
    {
        // publish here
        auto msg = encoders.GetVelocities();
        msg.header.seq = seq++;

        publisher.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
    }

    return 0;
}
