#include "robot_wheel_speeds/DifferentialDriveEncoders.h"

DifferentialDriveEncoders::DifferentialDriveEncoders(
        int leftPinA,
        int leftPinB,
        int rightPinA,
        int rightPinB,
        int ticksPerRevolution,
        int wheelDiameterMm,
        int wheelAxisMm,
        const ros::Duration& velocityUpdateInterval
    )
    : m_leftWheel(leftPinA, leftPinB, ticksPerRevolution, wheelDiameterMm, velocityUpdateInterval),
      m_rightWheel(rightPinA, rightPinB, ticksPerRevolution, wheelDiameterMm, velocityUpdateInterval),
      m_wheelAxis(float(wheelAxisMm)/1000),
      m_keepThreadAlive(false)
{
}

DifferentialDriveEncoders::~DifferentialDriveEncoders()
{
    m_keepThreadAlive = false;
    m_encoderPollingThread.join();
}

void DifferentialDriveEncoders::Start()
{
    m_keepThreadAlive = true;
    m_encoderPollingThread = std::thread(
        [this]()
        {
            while(m_keepThreadAlive)
            {
                m_mutex.lock();

                m_timeNow = ros::Time::now();
                m_leftWheel.DoReading(m_timeNow);
                m_rightWheel.DoReading(m_timeNow);

                m_mutex.unlock();
            }
        });
}


robot_wheel_speeds::WheelVelocities DifferentialDriveEncoders::GetVelocities() const
{
    m_mutex.lock();

    robot_wheel_speeds::WheelVelocities msg;

    msg.header.stamp = m_timeNow;

    msg.left = m_leftWheel.GetVelocity();
    msg.right = m_rightWheel.GetVelocity();
    msg.velocity = calculateUnicycleVelocites(msg.left, msg.right);

    m_mutex.unlock();

    return msg;
}

geometry_msgs::Twist DifferentialDriveEncoders::calculateUnicycleVelocites(float left, float right) const
{
    geometry_msgs::Twist velocities;
    velocities.linear.x = 0.5 * (left + right);
    velocities.angular.z = (right - left)/m_wheelAxis;

    return velocities;
}
