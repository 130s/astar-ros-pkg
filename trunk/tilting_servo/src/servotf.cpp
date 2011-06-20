#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tilting_servo/servoAction.h>
#define pi 3.14159
void angleCallback(const tilting_servo::servoActionFeedbackConstPtr& feedback)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  double angle = (180 - feedback->feedback.angle) * pi / 180;
  transform.setRotation(tf::createQuaternionFromRPY(0,angle,0) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "laser_tilt_mount_link", "laser_tilt_link"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "servotf_broadcaster");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("/servo/feedback", 10, &angleCallback);

  ros::spin();
  return 0;
};
