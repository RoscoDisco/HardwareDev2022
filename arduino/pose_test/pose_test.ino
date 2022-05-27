/* 
 * rosserial::geometry_msgs::PoseArray Test
 * Sums an array, publishes sum 
 */

#include <ros.h>
#include <geometry_msgs/PoseStamped.h>


ros::NodeHandle nh;


bool set_; 


geometry_msgs::Pose sum_msg;
ros::Publisher p("sum", &sum_msg);

void messageCb(const geometry_msgs::PoseStamped& msg){
  sum_msg.position.x = 0;
  sum_msg.position.y = 0;
  sum_msg.position.z = 0;

  sum_msg.position.x += msg.pose.position.x;
  sum_msg.position.y += msg.pose.position.y;
  sum_msg.position.z += msg.pose.position.z;
 
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<geometry_msgs::PoseStamped> s("haptic/updated_pose",messageCb);

void setup()
{ 
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.subscribe(s);
  nh.advertise(p);
}

void loop()
{  
  p.publish(&sum_msg);
  nh.spinOnce();
  delay(10);
}
