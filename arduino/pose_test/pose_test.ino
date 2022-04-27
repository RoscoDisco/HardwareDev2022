/* 
 * rosserial::geometry_msgs::PoseArray Test
 * Sums an array, publishes sum 
 */

#include <ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>


ros::NodeHandle nh;


bool set_; 


geometry_msgs::Pose sum_msg;
ros::Publisher p("sum", &sum_msg);

void messageCb(const geometry_msgs::Pose& msg){
  sum_msg.position.x = 0;
  sum_msg.position.y = 0;
  sum_msg.position.z = 0;

  sum_msg.position.x += msg.position.x;
  sum_msg.position.y += msg.position.y;
  sum_msg.position.z += msg.position.z;
 
  digitalWrite(13, HIGH-digitalRead(13));   // blink the led
}

ros::Subscriber<geometry_msgs::Pose> s("poses",messageCb);

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
