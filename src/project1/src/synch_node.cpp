#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <dynamic_reconfigure/server.h>
#include <project1/ThresholdConfig.h>
#include "nav_msgs/Odometry.h"
#include "project1/DistanceComputer.h"
#include "project1/OutputMsg.h"


static struct Thresholds {
    double safeTh;
    double crashTh;
} th = {0.0, 0.0};

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

  project1::OutputMsg msg;

  if (isnan(srv.request.e1) || isnan(srv.request.n1) || isnan(srv.request.u1) ||
          isnan(srv.request.e2) || isnan(srv.request.n2) || isnan(srv.request.u2)) {
      ROS_INFO("Either the car GPS or the obs GPS was loose, sending nan to /car_info_topic.");
      msg.distance = nan("");
      msg.status = "Not computable";

  } else {
      if(client.call(srv))
      {
          double& dist = srv.response.dist;
          msg.distance = dist;
          if(dist > th.safeTh)
              msg.status = "Safe";
          else if (dist < th.crashTh)
              msg.status = "Crash";
          else
              msg.status = "Unsafe";
      }
      else
      {
          ROS_ERROR("Failed to call service distance_computer");
          return;
      }
  }
  pub_info.publish(msg);

}

void onParamChange (project1::ThresholdConfig &config, uint32_t level) {
    ROS_INFO("Reconfigure request: safe_th -> %f, crash_th -> %f", config.safe_th, config.crash_th);
    th.safeTh = config.safe_th;
    th.crashTh = config.crash_th;
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

  message_filters::Subscriber<nav_msgs::Odometry> carOdomSub(nh,"/car/odometry",1);
  message_filters::Subscriber<nav_msgs::Odometry> obsOdomSub(nh,"/obs/odometry",1);

  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, nav_msgs::Odometry> ApproxTimePolicy;

  message_filters::Synchronizer<ApproxTimePolicy> sync (ApproxTimePolicy(10), carOdomSub, obsOdomSub);

  sync.registerCallback(boost::bind(&onSync, boost::ref(client), boost::ref(srv), boost::ref(pub_info), _1, _2));

  //setting thresholds default values from launch file
  nh.getParam(ros::this_node::getName() + "/safeThDefVal", th.safeTh);
  nh.getParam(ros::this_node::getName() + "/crashThDefVal", th.crashTh);

  dynamic_reconfigure::Server<project1::ThresholdConfig> drServer;
  dynamic_reconfigure::Server<project1::ThresholdConfig>::CallbackType callback;

  callback = boost::bind(&onParamChange, _1, _2);
  drServer.setCallback(callback);

  ros::spin();

}
