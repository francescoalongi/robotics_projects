#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include "nav_msgs/Odometry.h"
#include <message_filters/time_synchronizer.h>
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

  if(client.call(srv))
  {
    double& dist = srv.response.dist;
    if(isnan(dist))
      return;
    else {
      project1::OutputMsg msg;
      msg.distance = dist;
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
  ros::ServiceClient client = nh.serviceClient<project1::DistanceComputerRequest>("distance_computer");
  project1::DistanceComputer srv;
  std::string car_info_topic;
  nh.getParam(ros::this_node::getName() + "/car_info_topic", car_info_topic);
  ros::Publisher pub_info = nh.advertise<project1::OutputMsg>(car_info_topic,1);

  message_filters::Subscriber<nav_msgs::Odometry> car_odom(nh,"/car/odometry",1);
  message_filters::Subscriber<nav_msgs::Odometry> obs_odom(nh,"/obs/odometry",1);
  message_filters::TimeSynchronizer<nav_msgs::Odometry,nav_msgs::Odometry> sync(car_odom, obs_odom,10); //queue length???
  sync.registerCallback(boost::bind(&onSync, boost::ref(client), boost::ref(srv), boost::ref(pub_info), _1, _2));

}
