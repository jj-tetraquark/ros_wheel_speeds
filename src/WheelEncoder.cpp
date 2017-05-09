#include "robot_wheel_speeds/WheelEncoder.h"

#include <wiringPi.h>
#include <cmath>

WheelEncoder::WheelEncoder(int pinA,
                           int pinB,
                           unsigned int ticksPerRevolution,
                           unsigned int wheelDiameter,
                           const ros::Duration& updateInterval)
    : m_pinA(pinA),
      m_pinB(pinB),
      m_ticksPerRevolution(ticksPerRevolution),
      m_wheelCircumference(2 * M_PI * wheelDiameter),
      m_velocityUpdateInterval(updateInterval),
      m_prevValueA(LOW),
      m_lastUpdateTime(ros::Time(0)),
      m_direction(0),
      m_ticks(0),
      m_velocity(0)
{
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
}

void WheelEncoder::DoReading(const ros::Time& timenow)
{
    const int valueA = digitalRead(m_pinA);
    const int valueB = digitalRead(m_pinB);

    if (m_prevValueA == LOW && valueA == HIGH) // rising edge
    {
        m_direction = (valueB == HIGH) ? 1 : -1;
        m_ticks++;
    }
    m_prevValueA = valueA;
}

void WheelEncoder::MaybeUpdateVelocity(const ros::Time& timenow)
{
    auto dt = m_lastUpdateTime - timenow;
    if (m_lastUpdateTime.isValid() &&
        dt > m_velocityUpdateInterval)
    {
        UpdateVelocity(dt);
    }
    m_lastUpdateTime = timenow;
}


void WheelEncoder::UpdateVelocity(const ros::Duration& dt)
{
    float distance = float(m_ticks)/m_ticksPerRevolution * (m_wheelCircumference/1000.0); //mm to m
    m_velocity = distance / dt.toSec();
}
