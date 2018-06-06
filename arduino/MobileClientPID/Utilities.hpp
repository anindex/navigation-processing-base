#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include "BaseParameter.h"
#include "Motor.hpp"
#include <geometry_msgs/Twist.h>

struct Ticks
{
public:
  double tLeft;
  double tRight;

  Ticks() : tLeft(0.0), tRight(0.0) {}
  Ticks(double l, double r) : tLeft(l), tRight(r) {}
};

static Ticks SetPoint;
static Ticks Input;
static Ticks Output; // not ticks, it is PWM signal


void vel_convert(const geometry_msgs::Twist& msg, Ticks* res) //need consider
{
  double deltaX = msg.linear.x * ENCODER_PUB_PERIOD; // in milimeters
  double deltaTh = msg.angular.z * ENCODER_PUB_PERIOD / 1000.0; // in rad

  res->tLeft = deltaX * TICK_PER_ROUND / (PI * WHEEL_DIAMETER); // setPoint encoder delta tick per period
  res->tRight = res->tLeft;

  double offsetOmega = deltaTh * (BASE_DIAMETER / 2.0) * TICK_PER_ROUND / (PI * WHEEL_DIAMETER);
  res->tLeft -= offsetOmega;
  res->tRight += offsetOmega;
}

void cmd_vel_callback(const geometry_msgs::Twist& msg)
{
    vel_convert(msg, &SetPoint);
}


#endif // end UTILITIES_HPP
