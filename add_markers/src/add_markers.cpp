// %Tag(FULLTEXT)%
// %Tag(INCLUDES)%
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <std_msgs/UInt8.h>
// %EndTag(INCLUDES)%

uint8_t robot_status = 0;

double PickupX = 4;
double PickupY = -11;
double DropoffX = 5;
double DropoffY = -19;

void robot_status_callback(const std_msgs::UInt8::ConstPtr& msg){
  robot_status = msg->data;
}

// %Tag(INIT)%
int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber robot_status_sub = n.subscribe("/robot_status", 1, robot_status_callback);
// %EndTag(INIT)%

  // Set our initial shape type to be a cube
// %Tag(SHAPE_INIT)%
  uint32_t shape = visualization_msgs::Marker::CUBE;
// %EndTag(SHAPE_INIT)%

// %Tag(MARKER_INIT)%
  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
// %EndTag(MARKER_INIT)%

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
// %Tag(NS_ID)%
    marker.ns = "basic_shapes";
    marker.id = 0;
// %EndTag(NS_ID)%

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
// %Tag(TYPE)%
    marker.type = shape;
// %EndTag(TYPE)%

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
// %Tag(POSE)%
      marker.pose.position.z = 0;
      marker.pose.orientation.x = 0.0;
      marker.pose.orientation.y = 0.0;
      marker.pose.orientation.z = 0.0;
      marker.pose.orientation.w = 1.0;
// %EndTag(POSE)%

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
// %Tag(SCALE)%
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
// %EndTag(SCALE)%

    // Set the color -- be sure to set alpha to something non-zero!
// %Tag(COLOR)%
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
// %EndTag(COLOR)%

// %Tag(LIFETIME)%
      marker.lifetime = ros::Duration();
// %EndTag(LIFETIME)%

    // Publish the marker
// %Tag(PUBLISH)%
      while (marker_pub.getNumSubscribers() < 1)
      {
        if (!ros::ok())
        {
          return 0;
        }
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
      }

    switch (robot_status)
    {
      case 0:
      {
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = PickupX;
        marker.pose.position.y = PickupY;
        marker_pub.publish(marker);
        ROS_INFO("add marker at pickup");
        break;
      }
      case 1:
      {
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
        ROS_INFO("marker picked up");
        break;
      }
      case 2:
      {
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = DropoffX;
        marker.pose.position.y = DropoffY;
        marker_pub.publish(marker);
        ROS_INFO("add markers at dropoff");
        ros::Duration(5.0).sleep();
        return 0;
      }
    }

    ros::spinOnce();
  }
}
// %EndTag(FULLTEXT)%
