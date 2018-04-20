#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <processing_base/base_parameter.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_link_broadcaster");

  ros::NodeHandle n;
  tf::TransformBroadcaster laser_br;
  ros::Rate rate(10.0);

  while(ros::ok())
  {
    laser_br.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(LASER_TRANSLATE_X, LASER_TRANSLATE_Y, LASER_TRANSLATE_Z)),
                                                ros::Time::now(),
                                                "base_link",
                                                "laser"));

    rate.sleep();
  }
}
