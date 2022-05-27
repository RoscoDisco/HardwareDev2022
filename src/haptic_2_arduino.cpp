// USAGE. Node for the conversion of haptic node information, process it and send it to the arduino

// ROS includes
#include <ros/ros.h>

// Message includes
#include <geometry_msgs/PoseStamped.h>


class Haptic_2_Arduino{
  private:
  public:
    // ROS node handler
    ros::NodeHandle pnh_;
    ros::NodeHandle nh_;

    // ROS publisher
    ros::Publisher pose_pub_;

    // ROS subscriber
    ros::Subscriber haptic_pose_;

    Haptic_2_Arduino(){
        // ROS params
        pnh_ = ros::NodeHandle("~");


        // Setup publisher
        pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/haptic/updated_pose", 1);
        // Setup subscriber
        haptic_pose_ = nh_.subscribe("Left/Pose", 1, &Haptic_2_Arduino::Pose_Callback, this);
    }

    void Pose_Callback(const geometry_msgs::PoseStamped::ConstPtr& p_){
      geometry_msgs::PoseStamped updated_pose;
      ROS_INFO("x: %f, y: %f, z: %f", p_->pose.position.x, p_->pose.position.y, p_->pose.position.z);

      // Update pose
      updated_pose.pose.position.x = p_->pose.position.x;
      updated_pose.pose.position.y = p_->pose.position.y;
      updated_pose.pose.position.z = p_->pose.position.z;

      updated_pose.pose.position.x -=0.009;
      updated_pose.pose.position.y -=0.00995;

      updated_pose.pose.position.x *= -2000;
      updated_pose.pose.position.y *= -2000;
      updated_pose.pose.position.z *= 1000;


      ROS_INFO("x: %f, y: %f, z: %f", updated_pose.pose.position.x,updated_pose.pose.position.y, updated_pose.pose.position.z);

      pose_pub_.publish(updated_pose);
    }
};


int main(int argc, char  *argv[]) {
  ros::init(argc, argv, "haptic_2_arduino");

  Haptic_2_Arduino h2a;
  ros::spin();
  return 0;
}
