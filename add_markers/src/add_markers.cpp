#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Empty.h>

// Euler-to-Quaternion conversion from https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
struct Quaternion{
    double w, x, y, z;
};

const double PI = 3.141592653589793238463;
const double PICKUP_DIST_THRESHOLD = 0.5;

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

void add_marker(ros::Publisher& marker_pub, double x, double y, double yaw){
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers_marker";
    marker.id = 0;

    // Set the marker type
    marker.type = visualization_msgs::Marker::CUBE;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;

    Quaternion q = to_quaternion((yaw / 180) * PI, 0, 0);

    marker.pose.orientation.x = q.x;
    marker.pose.orientation.y = q.y;
    marker.pose.orientation.z = q.z;
    marker.pose.orientation.w = q.w;

    // Set the scale of the marker, in meters
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return;
      }
      ROS_WARN_ONCE("Waiting for subscribers to the marker...");
      sleep(1);
    }

    marker_pub.publish(marker);
}

void delete_marker(ros::Publisher& marker_pub){
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers_marker";
    marker.id = 0;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::DELETE;

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return;
      }
      ROS_WARN_ONCE("Waiting for subscribers to the marker...");
      sleep(1);
    }

    marker_pub.publish(marker);
}

class Marker{
private:
    const static double MAX_PICKUP_DISTANCE = 0.25;

    ros::Publisher market_pub;
    ros::Subscriber odom_sub, pickup_sub;

    double x_marker, y_marker;
    double x_robot, y_robot;
    bool is_carried;

    bool can_pickup(){
      // Checks whether robot is close enough to target for pickup
      double dx = x_marker - x_robot;
      double dy = y_marker - y_robot;
 
      return dx*dx + dy*dx <= MAX_PICKUP_DISTANCE*MAX_PICKUP_DISTANCE;
    }

public:
    Marker(ros::NodeHandle n, double x, double y, bool is_carried_init){
      market_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
      odom_sub = n.subscribe("/odom", 1, &Marker::odom_callback, this);
      pickup_sub = n.subscribe("/pickup", 1, &Marker::pickup_callback, this);

      x_marker = x;
      y_marker = y;

      x_robot = 0;
      y_robot = 0;

      is_carried = is_carried_init;

      if (!is_carried) {
        // Marker is not carried, add it to viz
        add_marker(market_pub, x_marker, y_marker, 0);
      }
    }

    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg){
      // Callback for odometry readings

      // Update last known robot location
      x_robot = msg->pose.pose.position.x;
      y_robot = msg->pose.pose.position.y;

      if (is_carried) {
        // If carried, update marker location
        x_marker = x_robot;
        y_marker = y_robot;
      }
    }

    void pickup_callback(const std_msgs::Empty::ConstPtr& msg){
      // Callback whenever marker is picked up or dropped off
      if (is_carried || can_pickup()){
         is_carried = !is_carried;
         if (is_carried){
            // Marker is now carried, remove it
            delete_marker(market_pub);
         } else {
            // Marker has been dropped, add it back
            add_marker(market_pub, x_marker, y_marker, 0);
         }
      }
    }
};

int main(int argc, char** argv){
  // Initialize the add_markers node
  ros::init(argc, argv, "add_markers");

  ros::NodeHandle n;

  // Initialize the marker
  Marker m(n, 5.5, 0.0, false);

  // Spin
  ros::spin();
}
