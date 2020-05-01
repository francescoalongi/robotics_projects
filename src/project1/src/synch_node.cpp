#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include "nav_msgs/Odometry.h"
#include "project1/DistanceComputer.h"
#include "project1/OutputMsg.h"

void onSync(ros::ServiceClient& client, project1::DistanceComputer& srv, ros::Publisher& pub_info, const nav_msgs::OdometryConstPtr& msg0, const nav_msgs::OdometryConstPtr& msg1)
{

  srv.request.e1 = msg0.get()->pose.pose.position.x;
  srv.request.n1 = msg0.get()->pose.pose.position.y;
  srv.request.u1 = msg0.get()->pose.pose.position.z;

  srv.request.e2 = msg1.get()->pose.pose.position.x;
  srv.request.n2 = msg1.get()->pose.pose.position.y;
  srv.request.u2 = msg1.get()->pose.pose.position.z;
  ROS_INFO("synch_node callback called! x1->%f, y1->%f, z1->%f, x2->%f, y2->%f, z2->%f",
           srv.request.e1, srv.request.n1, srv.request.u1, srv.request.e2, srv.request.n2, srv.request.u2);

  if(client.call(srv))
  {
    double& dist = srv.response.dist;
    project1::OutputMsg msg;
    msg.distance = dist ;
    if(isnan(dist))
      msg.status = "nan";
    else {
      if(dist > 5)
        msg.status = "Safe";
      else if (dist < 1)
        msg.status = "Crash";
      else
        msg.status = "Unsafe";

      pub_info.publish(msg);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service distance_computer");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "synch_node");
  ROS_INFO("synch_node started...\n");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<project1::DistanceComputer>("compute_distance",100);
  project1::DistanceComputer srv;
//  std::string car_info_topic;
//  nh.getParam(ros::this_node::getName() + "/car_info_topic", car_info_topic);
  ros::Publisher pub_info = nh.advertise<project1::OutputMsg>("/car_info_topic",1);

  message_filters::Subscriber<nav_msgs::Odometry> car_odom_sub(nh,"/car/odometry",1);
  message_filters::Subscriber<nav_msgs::Odometry> obs_odom_sub(nh,"/obs/odometry",1);

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> ApproxTimePolicy;

  message_filters::Synchronizer<ApproxTimePolicy> sync (ApproxTimePolicy(10), car_odom_sub, obs_odom_sub);

  sync.registerCallback(boost::bind(&onSync, boost::ref(client), boost::ref(srv), boost::ref(pub_info), _1, _2));
  ros::spin();

}
