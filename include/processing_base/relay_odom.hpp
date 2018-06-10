#ifndef RELAY_ODOM_HPP
#define RELAY_ODOM_HPP

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point32.h>

#include <tf/transform_broadcaster.h>

#include <processing_base/base_parameter.h>
#include <boost/function.hpp>
#include <boost/bind.hpp>

void callback(const geometry_msgs::Point32::ConstPtr& msg, ros::Publisher& odomPub, tf::TransformBroadcaster& odom_broadcaster, float& x, float& y, float& theta)
{
  float dleft = WHEEL_DIAMETER * PI * msg->x / TICK_PER_ROUND; // in mm
  float dright = WHEEL_DIAMETER * PI * msg->y / TICK_PER_ROUND;
  float deltaTheta = (dright - dleft) / BASE_DIAMETER;

  float dcenter = (dleft + dright) / 2.0;

  theta += deltaTheta;
  //constrain theta on [0, 2pi]
  if (theta > 2 * PI) theta -= 2 * PI;
  if (theta < 0.0) theta += 2 * PI;

  float deltaX = dcenter * cos(theta) / 1000.0; // change to meters
  float deltaY = dcenter * sin(theta) / 1000.0; // change to meters

  float vx = deltaX / (msg->z / 1000.0); // change ms to seconds (m / s)
  float vy = deltaY / (msg->z / 1000.0); // change ms to seconds (mm /s)
  float omegaTheta = deltaTheta / (msg->z / 1000.0); // rad/s

  x += deltaX;
  y += deltaY;

  ros::Time current_time = ros::Time::now();
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = x;
  odom_trans.transform.translation.y = y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  odom_broadcaster.sendTransform(odom_trans);

  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "odom";
  //set the position
  odom.pose.pose.position.x = x;
  odom.pose.pose.position.y = y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  //set the velocity
  odom.child_frame_id = "base_link";
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = omegaTheta;

  odomPub.publish(odom);
}

class OdometryEncoderRelay
{
private:
  ros::NodeHandle n;
  ros::Publisher odomPub;
  ros::Subscriber encoderSub;
  tf::TransformBroadcaster odom_broadcaster;

  float x;
  float y;
  float theta;

public:
  OdometryEncoderRelay(std::string pub_name, std::string sub_name) : x(0.0f), y(0.0f), theta(0.0f)
  {
    odomPub = n.advertise<nav_msgs::Odometry>(pub_name, 50);

    boost::function<void (const geometry_msgs::Point32::ConstPtr&)> binded_callback = boost::bind(callback, _1, this->odomPub,
                                                                                                                 this->odom_broadcaster,
                                                                                                                 this->x,
                                                                                                                 this->y,
                                                                                                                 this->theta);
    encoderSub = n.subscribe<geometry_msgs::Point32> (sub_name, 100, binded_callback); // buffer 100 msgs
  }
};



#endif // end RELAY_ODOM_HPP
