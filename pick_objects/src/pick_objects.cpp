#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Empty.h>


// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// Euler-to-Quaternion conversion from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
struct Quaternion{
    double w, x, y, z;
};

const double PI = 3.141592653589793238463;

Quaternion to_quaternion(double yaw, double pitch, double roll){
    // Abbreviations for the various angular functions
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    Quaternion q;
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;

    return q;
}

void move_to_waypoint(MoveBaseClient& ac, double x, double y, double yaw, bool verbose, const char* msg_success){
  move_base_msgs::MoveBaseGoal goal;

  // Set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.position.z = 0.0;

  Quaternion q = to_quaternion((yaw / 180) * PI, 0, 0);

  goal.target_pose.pose.orientation.x = q.x;
  goal.target_pose.pose.orientation.y = q.y;
  goal.target_pose.pose.orientation.z = q.z;
  goal.target_pose.pose.orientation.w = q.w;

   // Send the goal position and orientation for the robot to reach
  if (verbose)
    ROS_INFO("Sending waypoint: {x:%f, y:%f, yaw:%f}", x, y, yaw);
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    if (verbose) {
      ROS_INFO("Reached waypoint: {x:%f, y:%f, yaw:%f}", x, y, yaw);
    }
    ROS_INFO("%s", msg_success);
  }
  else {
    ROS_INFO("Failed to reach waypoint: {x:%f, y:%f, yaw:%f}", x, y, yaw);
  }
}

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  // Create pickup / dropoff publisher
  ros::NodeHandle n;
  ros::Publisher pickup_pub = n.advertise<std_msgs::Empty>("pickup", 1);

  // Tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Move to first destination
  move_to_waypoint(ac, 5.5, 0, 0, false, "Reached drop-off zone");

  // Send pickup message
  std_msgs::Empty empty_msg;
  pickup_pub.publish(empty_msg);

  // Wait for 5 sec
  ros::Duration(5.0).sleep();

  // Move to second destination
  move_to_waypoint(ac, -1, -2, 225, false, "Reached pickup zone");

  // Send dropoff message
  pickup_pub.publish(empty_msg);

  return 0;
}
