#ifndef UTILITIES_HPP
#define UTILITIES_HPP

#include "BaseParameter.h"
#include "Motor.hpp"
#include <geometry_msgs/Twist.h>

#define TEMP_MAX_PWM   200
#define TEMP_AVER_PWM  145
#define TEMP_MIN_PWM   100

struct Ticks
{
public:
  float tLeft;
  float tRight;

  Ticks() : tLeft(0.0), tRight(0.0) {}
  Ticks(float l, float r) : tLeft(l), tRight(r) {}
};

static Ticks setPoint;

Ticks vel_convert(const geometry_msgs::Twist& msg) //need consider
{
  float deltaX = msg.linear.x * ENCODER_PUB_PERIOD; // in milimeters
  float deltaTh = msg.angular.z * ENCODER_PUB_PERIOD / 1000.0; // in rad

  Ticks res(0.0, 0.0);
  res.tLeft = deltaX * TICK_PER_ROUND / (PI * WHEEL_DIAMETER); // setPoint encoder delta tick per period
  res.tRight = res.tLeft;

  float offsetOmega = deltaTh * (BASE_DIAMETER / 2.0) * TICK_PER_ROUND / (PI * WHEEL_DIAMETER);
  res.tLeft -= offsetOmega;
  res.tRight += offsetOmega;

  return res;
}

void cmd_vel_callback(const geometry_msgs::Twist& msg)
{
    setPoint = vel_convert(msg);

    //naive direct control
    int left = (int) setPoint.tLeft;
    int right = (int) setPoint.tRight;

    if(left > 0)
    {
      if (left == 214)
      {
        motor::left_wheel_forward(TEMP_MAX_PWM);
      }
      else if(left == 183)
      {
        motor::left_wheel_forward(TEMP_AVER_PWM);
      }
      else if(left == 31)
      {
        motor::left_wheel_forward(TEMP_MIN_PWM);
      }
    }
    else
    {
      if (left == -214)
      {
        motor::left_wheel_backward(TEMP_MAX_PWM);
      }
      else if(left == -183)
      {
        motor::left_wheel_backward(TEMP_AVER_PWM);
      }
      else if(left == -31)
      {
        motor::left_wheel_backward(TEMP_MIN_PWM);
      }
    }

    if(right > 0)
    {
      if (right == 214)
      {
        motor::right_wheel_forward(TEMP_MAX_PWM);
      }
      else if(right == 183)
      {
        motor::right_wheel_forward(TEMP_AVER_PWM);
      }
      else if(right == 31)
      {
        motor::right_wheel_forward(TEMP_MIN_PWM);
      }
    }
    else
    {
      if (right == -214)
      {
        motor::right_wheel_backward(TEMP_MAX_PWM);
      }
      else if(right == -183)
      {
        motor::right_wheel_backward(TEMP_AVER_PWM);
      }
      else if(right == -31)
      {
        motor::right_wheel_backward(TEMP_MIN_PWM);
      }
    }

    if(left == 0 && right == 0)
    {
      motor::emergency_stop();
    }
    
}



#endif // end UTILITIES_HPP
