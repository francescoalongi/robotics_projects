#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
#include <math.h>  
#include <tf/transform_broadcaster.h>

class ecef_to_enu 
{

    private: 
        ros::NodeHandle n;
        ros::Subscriber ecef_pose_sub;
        ros::Publisher enu_pose_pub;
        // the below publisher is only used for debug purposes
        ros::Publisher enu_pose_pub_debug;
        tf::TransformBroadcaster br;

    public:
        ecef_to_enu() {
            std::string ecef_topic;
            std::string enu_topic;
            std::string node_name;
            n.getParam(ros::this_node::getName() + "/ecef_topic", ecef_topic);
            n.getParam(ros::this_node::getName() + "/enu_topic", enu_topic);
            ecef_pose_sub = n.subscribe(ecef_topic,1000,&ecef_to_enu::ecef_to_enu_converter, this);
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
            float deg_to_rad = 0.0174533;

            // input data from msg
            float latitude = msg->latitude;
            float longitude = msg->longitude;
            float h = msg->altitude;

            // fixed position
            float latitude_init;
            float longitude_init;
            float h0;
            n.getParam("latitude_init", latitude_init);
            n.getParam("longitude_init", longitude_init);
            n.getParam("altitude_init", h0);

            //lla to ecef
            float lamb = deg_to_rad*(latitude);
            float phi = deg_to_rad*(longitude);
            float s = sin(lamb);
            float N = a / sqrt(1 - e_sq * s * s);

            float sin_lambda = sin(lamb);
            float  cos_lambda = cos(lamb);
            float  sin_phi = sin(phi);
            float  cos_phi = cos(phi);

            float  x = (h + N) * cos_lambda * cos_phi;
            float  y = (h + N) * cos_lambda * sin_phi;
            float  z = (h + (1 - e_sq) * N) * sin_lambda;

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

            float  x0 = (h0 + N) * cos_lambda * cos_phi;
            float  y0 = (h0 + N) * cos_lambda * sin_phi;
            float  z0 = (h0 + (1 - e_sq) * N) * sin_lambda;

            float xd = x - x0;
            float  yd = y - y0;
            float  zd = z - z0;

            float  xEast = -sin_phi * xd + cos_phi * yd;
            float  yNorth = -cos_phi * sin_lambda * xd - sin_lambda * sin_phi * yd + cos_lambda * zd;
            float  zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd;

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
  	ecef_to_enu e_to_e_instance;
    ros::spin();
    
    return 0;
}

