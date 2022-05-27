/*
 * rosserial::geometry_msgs::PoseArray Test
 * Sums an array, publishes sum
 */

// INclude at start
#include <ros.h>
#include <geometry_msgs/PoseStamped.h>


ros::NodeHandle nh;

void messageCb(const geometry_msgs::PoseStamped& msg){
  x = msg.pose.position.x;
  y = msg.pose.position.y;

  return;
}

ros::Subscriber<geometry_msgs::PoseStamped> s("haptic/updated_pose",messageCb);

void setup()
{
  // ROS setup
  nh.initNode();
  nh.subscribe(s);
}

void loop()
{
  nh.spinOnce();

}
