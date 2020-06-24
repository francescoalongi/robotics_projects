#include "ros/ros.h"
#include "cmath"
#include "project1/DistanceComputer.h"


bool compute_distance(project1::DistanceComputer::Request& req,
                      project1::DistanceComputer::Response& res)
{
  if((isnan(req.e1) && isnan(req.n1) && isnan(req.u1)) || (isnan(req.e2) && isnan(req.n2) && isnan(req.u2)))
    res.dist = nan("");
  else
    res.dist = sqrt(pow(req.e1 - req.e2,2) + pow(req.n1 - req.n2,2) + pow(req.u1 - req.u2,2));
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distance_computer");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("compute_distance", compute_distance);
  ROS_INFO("Ready to compute the distance.");
  ros::spin();

  return 0;
}
