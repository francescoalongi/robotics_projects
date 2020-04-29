#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <math.h>  
#include <tf/transform_broadcaster.h>

class ECEFtoENU
{

    private: 
        ros::NodeHandle n;
        ros::Subscriber ecef_pose_sub;
        ros::Publisher enu_pose_pub;
        // the below publisher is only used for debug purposes
        ros::Publisher enu_pose_pub_debug;
        tf::TransformBroadcaster br;

    public:
        ECEFtoENU() {
            std::string ecef_topic;
            std::string enu_topic;
            std::string node_name;
            n.getParam(ros::this_node::getName() + "/ecef_topic", ecef_topic);
            n.getParam(ros::this_node::getName() + "/enu_topic", enu_topic);
            ecef_pose_sub = n.subscribe(ecef_topic,1000,&ECEFtoENU::ecef_to_enu_converter, this);
            enu_pose_pub = n.advertise<nav_msgs::Odometry>(enu_topic, 1);
            enu_pose_pub_debug = n.advertise<nav_msgs::Odometry>(enu_topic + "_debug", 1);
        }

        void ecef_to_enu_converter(const sensor_msgs::NavSatFix::ConstPtr& msg) {

            std::string child_frame_id;
            n.getParam(ros::this_node::getName() + "/child_frame_id", child_frame_id);

            //TODO: handle scenario in which GPS is absent (0,0,0), in such a case, publish a NaN

            ROS_INFO("%s input position: [%f,%f, %f]", child_frame_id.c_str(), msg->latitude, msg->longitude,msg->altitude);
            // fixed values
            double a = 6378137;
            double b = 6356752.3142;
            double f = (a - b) / a;
            double e_sq = f * (2-f);
            double deg_to_rad = 0.0174533;

            // input data from msg
            double latitude = msg->latitude;
            double longitude = msg->longitude;
            double h = msg->altitude;

            // fixed position
            double latitude_init;
            double longitude_init;
            double h0;
            n.getParam("latitude_init", latitude_init);
            n.getParam("longitude_init", longitude_init);
            n.getParam("altitude_init", h0);

            //lla to ecef
            double lamb = deg_to_rad*(latitude);
            double phi = deg_to_rad*(longitude);
            double s = sin(lamb);
            double N = a / sqrt(1 - e_sq * s * s);

            double sin_lambda = sin(lamb);
            double  cos_lambda = cos(lamb);
            double  sin_phi = sin(phi);
            double  cos_phi = cos(phi);

            double  x = (h + N) * cos_lambda * cos_phi;
            double  y = (h + N) * cos_lambda * sin_phi;
            double  z = (h + (1 - e_sq) * N) * sin_lambda;

            ROS_INFO("%s ECEF position: [%f,%f, %f]",child_frame_id.c_str(), x, y,z);

            // ecef to enu
            lamb = deg_to_rad*(latitude_init);
            phi = deg_to_rad*(longitude_init);
            s = sin(lamb);
            N = a / sqrt(1 - e_sq * s * s);

            sin_lambda = sin(lamb);
            cos_lambda = cos(lamb);
            sin_phi = sin(phi);
            cos_phi = cos(phi);

            double  x0 = (h0 + N) * cos_lambda * cos_phi;
            double  y0 = (h0 + N) * cos_lambda * sin_phi;
            double  z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

            double xd = x - x0;
            double  yd = y - y0;
            double  zd = z - z0;

            double  xEast = -sin_phi * xd + cos_phi * yd;
            double  yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
            double  zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

            nav_msgs::Odometry odom_msg;
            ros::Time current_time = ros::Time::now();

            // setting the car_odom_msg parameters
            odom_msg.header.stamp = current_time;
            odom_msg.header.frame_id = "world";

            odom_msg.pose.pose.position.x = xEast;
            odom_msg.pose.pose.position.y = yNorth;
            odom_msg.pose.pose.position.z = zUp;

            odom_msg.child_frame_id = child_frame_id;

            enu_pose_pub.publish(odom_msg);

            tf::Transform transform;
            transform.setOrigin( tf::Vector3(xEast, yNorth, zUp) );
            tf::Quaternion q;
            q.setRPY(0, 0, 0);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", child_frame_id));


            // DEBUG odom_msg
            nav_msgs::Odometry d_odom_msg;
            d_odom_msg.header.stamp = current_time;
            d_odom_msg.header.frame_id = "world";

            d_odom_msg.pose.pose.position.x = xEast/100;
            d_odom_msg.pose.pose.position.y = yNorth/100;
            d_odom_msg.pose.pose.position.z = zUp/100;

            d_odom_msg.child_frame_id = child_frame_id + "_debug";

            enu_pose_pub_debug.publish(d_odom_msg);

            ROS_INFO("%s ENU position: [%f,%f, %f]", child_frame_id.c_str(), xEast, yNorth,zUp);

        }

};



int main(int argc, char **argv){
  	
    ros::init(argc, argv, "ecef_to_enu");
    ECEFtoENU eCEFtoENU;
    ros::spin();
    
    return 0;
}

