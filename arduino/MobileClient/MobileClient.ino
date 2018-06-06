#include <ros.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Twist.h>

#include <Encoder.h>

#include "BaseParameter.h"
#include "Utilities.hpp"
#include "Motor.hpp"

ros::NodeHandle  nh;

geometry_msgs::Point32 currTicks;
ros::Publisher encoderPub("/encoder_ticks", &currTicks);

ros::Subscriber<geometry_msgs::Twist> velSub("/cmd_vel", &cmd_vel_callback);

Encoder knobLeft(2, 11);
Encoder knobRight(3, 12);

long prevLeft  = 0;
long prevRight = 0;

long publisher_timer = ENCODER_PUB_PERIOD;

void setup()
{ 
  nh.initNode();
  nh.advertise(encoderPub);
  nh.subscribe(velSub);

  motor::motorInit();
}

void loop()
{
  long now = millis();
  long newLeft, newRight;
  if (now >= publisher_timer) 
  { 
    newLeft = knobLeft.read();
    newRight = -knobRight.read(); // the right wheel rotates reversely

    // avoiding tick overflow
    if (abs(newLeft) > MAX_INT)
    {
      knobLeft.write(0);
    }
    if (abs(newRight) > MAX_INT)
    {
      knobRight.write(0);
    }

    currTicks.x = newLeft - prevLeft; // store delta tick left
    currTicks.y = newRight - prevRight; // store delta tick right
    currTicks.z = ENCODER_PUB_PERIOD; // store delta time

    prevLeft = newLeft;
    prevRight = newRight;

    encoderPub.publish(&currTicks);
    publisher_timer = millis() + ENCODER_PUB_PERIOD;

    nh.loginfo(String(setPoint.tLeft).c_str());
    nh.loginfo(String(setPoint.tRight).c_str());
  }
  
  nh.spinOnce();
}
