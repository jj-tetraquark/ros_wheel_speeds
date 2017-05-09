#pragma once

#include "robot_wheel_speeds/WheelEncoder.h"
#include "robot_wheel_speeds/WheelVelocities.h"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <thread>
#include <atomic>
#include <mutex>

class DifferentialDriveEncoders
{
public:
    DifferentialDriveEncoders(int leftPinA, int leftPinB, int rightPinA, int rightPinB,
                               int ticksPerRevolution, int wheelDiameterMm, int wheelAxisMm,
                               const ros::Duration& velocityUpdateInterval);
    ~DifferentialDriveEncoders();

    void Start();
    robot_wheel_speeds::WheelVelocities GetVelocities() const;

private:
    geometry_msgs::Twist calculateUnicycleVelocites(float left, float right) const;

    WheelEncoder m_leftWheel;
    WheelEncoder m_rightWheel;
    const float  m_wheelAxis; // spacing between wheels in metres

    ros::Time          m_timeNow;
    std::thread        m_encoderPollingThread;
    std::atomic_bool   m_keepThreadAlive;
    mutable std::mutex m_mutex;
};
