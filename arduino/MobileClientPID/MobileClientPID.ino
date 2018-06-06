#include <ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>

#include <Encoder.h>
#include <PID_v1.h>

#include "BaseParameter.h"
#include "Utilities.hpp"
#include "Motor.hpp"

ros::NodeHandle  nh;

geometry_msgs::Point32 currTicks;
ros::Publisher encoderPub("/encoder_ticks", &currTicks);

ros::Subscriber<geometry_msgs::Twist> velSub("/cmd_vel", &cmd_vel_callback);

Encoder knobLeft(2, 11);
Encoder knobRight(3, 12);

PID pidLeft(&Input.tLeft, &Output.tLeft, &SetPoint.tLeft, LKP, LKI, LKD, DIRECT); //using Proportional On Error mode
PID pidRight(&Input.tRight, &Output.tRight, &SetPoint.tRight, RKP, RKI, RKD, DIRECT);

double prevLeft  = 0;
double prevRight = 0;

long publisher_timer = ENCODER_PUB_PERIOD;

void setup()
{ 
  nh.initNode();
  nh.advertise(encoderPub);
  nh.subscribe(velSub);

  motor::motorInit();

  pidLeft.SetMode(AUTOMATIC);
  pidRight.SetMode(AUTOMATIC);

  pidLeft.SetOutputLimits(-255, 255);
  pidRight.SetOutputLimits(-255, 255);

  pidLeft.SetSampleTime(ENCODER_PUB_PERIOD); // same period with encoder output
  pidRight.SetSampleTime(ENCODER_PUB_PERIOD);
}

void loop()
{
  long now = millis();
  if (now >= publisher_timer) 
  { 
    double newLeft = knobLeft.read();
    double newRight = -knobRight.read(); // the right wheel rotates reversely

    // avoiding tick overflow
    if (abs(newLeft) > MAX_INT)
    {
      knobLeft.write(0);
    }
    if (abs(newRight) > MAX_INT)
    {
      knobRight.write(0);
    }

    Input.tLeft = newLeft - prevLeft;
    Input.tRight = newRight - prevRight;
    
    currTicks.x = Input.tLeft; // store delta tick left
    currTicks.y = Input.tRight; // store delta tick right
    currTicks.z = ENCODER_PUB_PERIOD; // store delta time

    prevLeft = newLeft;
    prevRight = newRight;

    encoderPub.publish(&currTicks);
    publisher_timer = millis() + ENCODER_PUB_PERIOD;

    pidLeft.Compute();
    pidRight.Compute();

    if(Output.tLeft >= 0)
    {
      motor::left_wheel_forward((int) Output.tLeft);
    }
    else
    {
      motor::left_wheel_backward((int) -Output.tLeft);
    }

    if(Output.tRight >= 0)
    {
      motor::right_wheel_forward((int) Output.tRight);
    }
    else
    {
      motor::right_wheel_backward((int) -Output.tRight);
    }
  }

  nh.spinOnce();
}
