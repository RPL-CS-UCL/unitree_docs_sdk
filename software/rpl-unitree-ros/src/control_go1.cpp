#include <string>
#include <stdlib.h>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

class Go1Controller {
public:
Go1Controller(ros::NodeHandle *nh, float dt = 0.002, float deadmans_switch_time=0.5) : 
  safety_(LeggedType::A1), 
  udp_(HIGHLEVEL, 8090, "192.168.123.161", 8082),
  deadmans_switch_time_(deadmans_switch_time)
  {
    cmd_vel_sub_ = nh->subscribe<geometry_msgs::Twist>("cmd_vel", 1, 
            &Go1Controller::cmd_vel_cb, this);
    
    ros::NodeHandle nhPrivate = ros::NodeHandle("~");
    nhPrivate.param<float>("max_abs_linear_vel", max_abs_linear_vel_, 0.5f);
    nhPrivate.param<float>("max_abs_angular_vel_", max_abs_angular_vel_, 0.4f);

    cmd_ = {0};
    udp_.InitCmdData(cmd_);
    state_ = {0};
    dt_ = dt;
    last_send_time_ = ros::Time::now();
  }

  void UDPRecv()
  {
    udp_.Recv();
    // could publish here
    udp_.GetRecv(state_);
    ROS_INFO("level flag: %d", state_.levelFlag);
    ROS_INFO("mode: %d", state_.mode);
    ROS_INFO("cartesian x: %f", state_.footPosition2Body[0].x);
    ROS_INFO("cartesian y: %f", state_.footPosition2Body[0].y);
    ROS_INFO("cartesian z: %f", state_.footPosition2Body[0].z);
	// typedef struct
	// {
	// 	uint8_t levelFlag;
	// 	uint16_t commVersion;
	// 	uint16_t robotID;
	// 	uint32_t SN;
	// 	uint8_t bandWidth;
	// 	uint8_t mode;
	// 	float progress;
	// 	IMU imu;
	// 	uint8_t gaitType;                  // 0.idle  1.trot  2.trot running  3.climb stair
	// 	float footRaiseHeight;             // (unit: m, default: 0.08m), foot up height while walking
	// 	float position[3];                 // (unit: m), from own odometry in inertial frame, usually drift
	// 	float bodyHeight;                  // (unit: m, default: 0.28m),
	// 	float velocity[3];                 // (unit: m/s), forwardSpeed, sideSpeed, rotateSpeed in body frame
	// 	float yawSpeed;                    // (unit: rad/s), rotateSpeed in body frame        
	// 	Cartesian footPosition2Body[4];    // foot position relative to body
	// 	Cartesian footSpeed2Body[4];       // foot speed relative to body
	// 	int8_t temperature[20];
	// 	BmsState bms;
	// 	int16_t footForce[4];
	// 	int16_t footForceEst[4];
	// 	uint8_t wirelessRemote[40];
	// 	uint32_t reserve;
	// 	uint32_t crc;
	// } HighState;                           // high level feedback


  }
  void UDPSend()
  {
    udp_.Send();
  }

  void CheckControl()
  {
    ros::Duration time_since_cmd_vel = ros::Time::now() - last_send_time_;
    if (time_since_cmd_vel > ros::Duration(deadmans_switch_time_))
    {
      cmd_.gaitType = 0;
      cmd_.speedLevel = 0;
      cmd_.footRaiseHeight = 0;
      cmd_.bodyHeight = 0;
      cmd_.euler[0]  = 0;
      cmd_.euler[1] = 0;
      cmd_.euler[2] = 0;
      cmd_.velocity[0] = 0;
      cmd_.velocity[1] = 0;
      cmd_.yawSpeed = 0;
      cmd_.reserve = 0;
      udp_.SetSend(cmd_);
    }
  }

private:
  void cmd_vel_cb(const geometry_msgs::Twist::ConstPtr& msg)
  {

    float linear_x_vel = msg->linear.x;
    if (abs(msg->linear.x) > max_abs_linear_vel_)
    {
      ROS_WARN("command vel linear x over absolute limit, changing it.");
      linear_x_vel = min(max(-max_abs_linear_vel_, (float)msg->linear.x), max_abs_linear_vel_);
    }

    float linear_y_vel = msg->linear.y;
    if (abs(msg->linear.y) > max_abs_linear_vel_)
    {
      ROS_WARN("command vel linear y over absolute limit, changing it.");
      linear_y_vel = min(max(-max_abs_linear_vel_, (float)msg->linear.y), max_abs_linear_vel_);
    }

    float angular_z_vel = msg->angular.z;
    if (abs(msg->angular.z) > max_abs_angular_vel_)
    {
      ROS_WARN("command vel angular z over absolute limit, changing it.");
      angular_z_vel = min(max(-max_abs_angular_vel_, (float)msg->angular.z), max_abs_angular_vel_);
    }


    // udp_.GetRecv(state_);
    cmd_.mode = 2;      // 0:idle, default stand      1:forced stand     2:walk continuously
    cmd_.gaitType = 1;
    cmd_.speedLevel = 0;
    cmd_.footRaiseHeight = 0;
    cmd_.bodyHeight = 0;
    cmd_.euler[0]  = 0;
    cmd_.euler[1] = 0;
    cmd_.euler[2] = 0;
    cmd_.velocity[0] = linear_x_vel;
    cmd_.velocity[1] = linear_y_vel;
    cmd_.yawSpeed = angular_z_vel;
    cmd_.reserve = 0;

    udp_.SetSend(cmd_);
    last_send_time_ = ros::Time::now();
  }


  ros::Time last_send_time_;
  ros::Subscriber cmd_vel_sub_;
  std::string robot_ip_;

  Safety safety_;
  UDP udp_;
  HighCmd cmd_;
  HighState state_;
  float dt_; // 0.001~0.01
  float deadmans_switch_time_;
  float max_abs_linear_vel_;
  float max_abs_angular_vel_;
  

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "go1_controller");
    ros::NodeHandle nh;

    float dt = 0.002;
    Go1Controller controller = Go1Controller(&nh, dt);

    LoopFunc loop_control("control_loop", dt,    boost::bind(&Go1Controller::CheckControl, &controller));
    LoopFunc loop_udpSend("udp_send",     dt, 3, boost::bind(&Go1Controller::UDPSend,      &controller));
    LoopFunc loop_udpRecv("udp_recv",     0.5, 3, boost::bind(&Go1Controller::UDPRecv,      &controller));

    loop_udpSend.start();
    loop_udpRecv.start();
    loop_control.start();

    ros::spin();

    loop_udpSend.shutdown();
    loop_udpRecv.shutdown();
    loop_control.shutdown();
    
}