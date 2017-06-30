#include <cmath>
#include <cstdlib>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <rover12_comm/rover12_comm.h>
#include <rover12_drivers/AutonomousMode.h>
#include <rover12_drivers/EncoderStatus.h>
#include <rover12_drivers/ServoControl.h>
#include <rover12_drivers/messenger.h>

class ControlSubscriber {
public:
  ControlSubscriber(ros::NodeHandle& nh, rover12_drivers::Messenger& messenger)
    : nh_(nh), messenger_(messenger) {
    ctrl_sub_ = nh_.subscribe("/control/command", 1, &ControlSubscriber::controlCallback, this);
    auto_sub_ = nh_.subscribe("/control/autonomous", 1, &ControlSubscriber::autonomousCallback, this);
  }

  void controlCallback(const rover12_drivers::ServoControl::ConstPtr& msg) {
    rover12_comm::ControlMsg control_msg;
    control_msg.data.steering_us = msg->steering_us;
    control_msg.data.throttle_us = msg->throttle_us;
    messenger_.send(control_msg);
  }

  void autonomousCallback(const rover12_drivers::AutonomousMode::ConstPtr& msg) {
    rover12_comm::EstopMsg estop_msg;
    estop_msg.data.autonomous = msg->autonomous;
    messenger_.send(estop_msg);
  }

private:
  ros::NodeHandle& nh_;
  rover12_drivers::Messenger& messenger_;
  ros::Subscriber ctrl_sub_;
  ros::Subscriber auto_sub_;
};

class EncoderPublisher {
public:
  explicit EncoderPublisher(ros::NodeHandle& nh)
    : nh_(nh),
      ticks_initialized_(false) {
    odom_pub_ = nh_.advertise<geometry_msgs::TwistWithCovarianceStamped>("/wheels/data", 1);
    encoder_pub_ = nh_.advertise<rover12_drivers::EncoderStatus>("/wheels/status", 1);
    estop_pub_ = nh_.advertise<rover12_drivers::AutonomousMode>("/estop", 1);

    geometry_msgs::TwistWithCovariance twist_cov_msg;

    // Covariance is 6x6: x, y, z, rx, ry, rz
    // Diagonal: 0.001, 0.001, -1, -1, -1, -1
    const double x_cov = 0.001;
    const double y_cov = 0.001;
    const double z_cov = -1.0;
    const double rx_cov = -1.0;
    const double ry_cov = -1.0;
    const double rz_cov = -1.0;
    twist_cov_msg.covariance[0] = x_cov;
    twist_cov_msg.covariance[1] = 0.0;
    twist_cov_msg.covariance[2] = 0.0;
    twist_cov_msg.covariance[3] = 0.0;
    twist_cov_msg.covariance[4] = 0.0;
    twist_cov_msg.covariance[5] = 0.0;
    twist_cov_msg.covariance[6] = 0.0;
    twist_cov_msg.covariance[7] = y_cov;
    twist_cov_msg.covariance[8] = 0.0;
    twist_cov_msg.covariance[9] = 0.0;
    twist_cov_msg.covariance[10] = 0.0;
    twist_cov_msg.covariance[11] = 0.0;
    twist_cov_msg.covariance[12] = 0.0;
    twist_cov_msg.covariance[13] = 0.0;
    twist_cov_msg.covariance[14] = z_cov;
    twist_cov_msg.covariance[15] = 0.0;
    twist_cov_msg.covariance[16] = 0.0;
    twist_cov_msg.covariance[17] = 0.0;
    twist_cov_msg.covariance[18] = 0.0;
    twist_cov_msg.covariance[19] = 0.0;
    twist_cov_msg.covariance[20] = 0.0;
    twist_cov_msg.covariance[21] = rx_cov;
    twist_cov_msg.covariance[22] = 0.0;
    twist_cov_msg.covariance[23] = 0.0;
    twist_cov_msg.covariance[24] = 0.0;
    twist_cov_msg.covariance[25] = 0.0;
    twist_cov_msg.covariance[26] = 0.0;
    twist_cov_msg.covariance[27] = 0.0;
    twist_cov_msg.covariance[28] = ry_cov;
    twist_cov_msg.covariance[29] = 0.0;
    twist_cov_msg.covariance[30] = 0.0;
    twist_cov_msg.covariance[31] = 0.0;
    twist_cov_msg.covariance[32] = 0.0;
    twist_cov_msg.covariance[33] = 0.0;
    twist_cov_msg.covariance[34] = 0.0;
    twist_cov_msg.covariance[35] = rz_cov;

    geometry_msgs::Twist twist_msg;

    geometry_msgs::Vector3 linear_msg;
    linear_msg.x = 0.0;
    linear_msg.y = 0.0;
    linear_msg.z = 0.0;

    geometry_msgs::Vector3 angular_msg;
    angular_msg.x = 0.0;
    angular_msg.y = 0.0;
    angular_msg.z = 0.0;

    twist_msg.linear = linear_msg;
    twist_msg.angular = angular_msg;

    twist_cov_msg.twist = twist_msg;

    speed_msg_.twist = twist_cov_msg;
  }

  void wheelEncCallback(const rover12_comm::WheelEncMsg& msg) {
    // Don't publish on first message, just initialize our ticks.
    if (!ticks_initialized_) {
      prev_ticks_ = msg.data;
      ticks_initialized_ = true;
      return;
    }

    // Compute instananeous velocity on each wheel based on previous tick
    // count.
    const int32_t rl_tick_delta = msg.data.count_rear_left - prev_ticks_.count_rear_left;
    const int32_t rr_tick_delta = msg.data.count_rear_right - prev_ticks_.count_rear_right;

    // TODO: Make these params (everything in meters).
    const double wheel_diameter = 0.102;
    const int num_stripes = 20.0;
    const double wheel_circumference = wheel_diameter * M_PI;
    const double meters_per_tick = wheel_circumference / (4.0 * num_stripes);

    // Translate tick counts to distances traveled.
    const double rl_distance = rl_tick_delta * meters_per_tick;
    const double rr_distance = rr_tick_delta * meters_per_tick;

    // Compute time delta based on difference in hardware timestamp.
    const uint32_t hw_timestamp_delta_ms = msg.data.hw_timestamp_ms - prev_ticks_.hw_timestamp_ms;
    const double time_delta = static_cast<double>(hw_timestamp_delta_ms) / 1000.0;
    const double rl_speed = rl_distance / time_delta;
    const double rr_speed = rr_distance / time_delta;

    // Averaging wheel speeds gives a pretty good estimate of true speed.
    const double avg_speed = (rl_speed + rr_speed) / 2.0;

    // Timestamp and identify reference frame.
    speed_msg_.header.stamp = ros::Time::now();
    speed_msg_.header.frame_id = "base_link";

    // Update instaneous forward speed.
    speed_msg_.twist.twist.linear.x = avg_speed;

    // Publish!
    odom_pub_.publish(speed_msg_);

    // Publish raw encoder ticks.
    rover12_drivers::EncoderStatus encoder_msg;
    encoder_msg.header.stamp = ros::Time::now();
    encoder_msg.header.frame_id = "base_link";
    encoder_msg.hw_timestamp_ms = msg.data.hw_timestamp_ms;
    encoder_msg.count_rear_left = msg.data.count_rear_left;
    encoder_msg.count_rear_right = msg.data.count_rear_right;
    encoder_pub_.publish(encoder_msg);

    // Save current tick counts.
    prev_ticks_ = msg.data;
  }

  void estopCallback(const rover12_comm::EstopMsg& msg) {
    rover12_drivers::AutonomousMode auto_msg;
    auto_msg.header.stamp = ros::Time::now();
    auto_msg.header.frame_id = "base_link";
    auto_msg.autonomous = msg.data.autonomous;
    estop_pub_.publish(auto_msg);
  }

private:
  ros::NodeHandle& nh_;
  ros::Publisher odom_pub_;
  ros::Publisher encoder_pub_;
  ros::Publisher estop_pub_;
  rover12_comm::WheelEnc prev_ticks_;
  geometry_msgs::TwistWithCovarianceStamped speed_msg_;
  bool ticks_initialized_;
};

int main(int argc, char* argv[]) {
  if (argc < 2) {
    ROS_ERROR_STREAM("Usage: " << argv[0] << " <serial device>");
    return EXIT_FAILURE;
  }

  // Initialize ROS.
  ros::init(argc, argv, "control_board_node");
  ros::NodeHandle nh;

  // Connect to the serial device.
  rover12_drivers::Messenger messenger(argv[1]);

  // Create publishers and subscribers.
  EncoderPublisher encoder_publisher(nh);
  ControlSubscriber control_subscriber(nh, messenger);

  // Set callbacks on message type.
  messenger.setWheelEncCallback([&encoder_publisher](const rover12_comm::WheelEncMsg& msg) {
    encoder_publisher.wheelEncCallback(msg);
  });
  messenger.setEstopCallback([&encoder_publisher](const rover12_comm::EstopMsg& msg) {
      encoder_publisher.estopCallback(msg);
  });

  // Spin on the messenger and ROS.
  while (ros::ok()) {
    messenger.spin();
    ros::spinOnce();
  }

  return EXIT_SUCCESS;
};
