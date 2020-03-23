#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

#include <cmath>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "points_and_lines");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

  float f = 0.0;


  tf::TransformListener listener; // add tf_listener

  geometry_msgs::Point p;

  visualization_msgs::Marker points, line_strip, line_list;
  points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "camera_odom_frame";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;




  points.id = 0;
  line_strip.id = 1;
  line_list.id = 2;



  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;



  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.02;
  points.scale.y = 0.02;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;
  line_list.scale.x = 0.1;



  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;

  while (ros::ok())
  {

  // initialize the tf
  tf::StampedTransform transform;
  try{
    listener.lookupTransform("camera_odom_frame", "camera_pose_frame",ros::Time(0), transform);

  // pass tf to the marker
    p.x = transform.getOrigin().x();
    p.y = transform.getOrigin().y();
    p.z = transform.getOrigin().z();

    points.points.push_back(p);
    line_strip.points.push_back(p);

    // The line list needs two points for each line
    line_list.points.push_back(p);
    p.z += 1.0;
    line_list.points.push_back(p);

    //pubslish
    marker_pub.publish(points);
    // marker_pub.publish(line_strip);
    // marker_pub.publish(line_list);

  }

  catch (tf::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
    continue;
  }




    r.sleep();

    f += 0.04;
  }
}
