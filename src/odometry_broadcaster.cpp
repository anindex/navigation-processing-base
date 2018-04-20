#include <ros/ros.h>
#include <processing_base/relay_odom.hpp>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odometry_broadcaster");

  OdometryEncoderRelay relay("odom", "encoder_ticks");

  ros::spin();

  return 0;
}
