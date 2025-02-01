#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

class PointcloudRepublisher {
public:
  PointcloudRepublisher(ros::NodeHandle *nh, tf2_ros::Buffer* tf_buffer, tf2_ros::TransformListener* listener)
  {

sensor_msgs::PointCloud2 cloud_in, cloud_out;

    pc_sub_ = nh->subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, 
            &PointcloudRepublisher::pc_callback, this);
    pc_pub_ = nh->advertise<sensor_msgs::PointCloud2>("registered_scan", 1);
    
    buffer_ = tf_buffer;
    tf_listener_ = listener;
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    nhPrivate.param<std::string>("global_frame", global_frame_, "map");
    nhPrivate.param<std::string>("sensor_frame", sensor_frame_, "velodyne");
    ROS_INFO("pc republisher: global frame %s", global_frame_.c_str());
    ROS_INFO("pc republisher: sensor_frame frame %s", sensor_frame_.c_str());
  }

private:
  void pc_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = buffer_->lookupTransform(global_frame_, sensor_frame_,
                               ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      return;
    }

    sensor_msgs::PointCloud2 new_cloud;
    tf2::doTransform(*msg, new_cloud, transformStamped);
    pc_pub_.publish(new_cloud);
  }


  ros::Subscriber pc_sub_;
  ros::Publisher pc_pub_;
  std::string global_frame_;
  std::string sensor_frame_;
  tf2_ros::Buffer* buffer_;
  tf2_ros::TransformListener* tf_listener_;
  

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_republisher");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    PointcloudRepublisher republisher = PointcloudRepublisher(&nh, &tfBuffer, &tfListener);
    ros::spin();
}