#ifndef WHEEL_ENCODER
#define WHEEL_ENCODER
#include <ros/ros.h>
#include <atomic>

class WheelEncoder
{
public:
    WheelEncoder(int pinA,
                 int pinB,
                 unsigned int ticksPerRevolution,
                 unsigned int wheelDiameter,
                 const ros::Duration& velocityUpdateInteravl);

    void DoReading(const ros::Time& timenow);

    float GetVelocity() const { return m_velocity; }

private:
    void MaybeUpdateVelocity(const ros::Time& timenow);
    void UpdateVelocity(const ros::Duration& dt);

    const int           m_pinA;
    const int           m_pinB;
    const unsigned int  m_ticksPerRevolution;
    const float         m_wheelCircumference;
    const ros::Duration m_velocityUpdateInterval;

    int                 m_prevValueA;
    ros::Time           m_lastUpdateTime;
    int                 m_direction;
    std::atomic_int     m_ticks;
    std::atomic<float>  m_velocity;
};

#endif
