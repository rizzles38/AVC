#include <cstdlib>
#include <iostream>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <rover12_comm/rover12_comm.h>
#include <rover12_drivers/ImuCalibration.h>
#include <rover12_drivers/messenger.h>
#include <sensor_msgs/Imu.h>

class SensorPublisher {
public:
  SensorPublisher() {
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/sensors/imu", 0);
    imu_cal_pub_ = nh_.advertise<rover12_drivers::ImuCalibration>("/sensors/imu_calibration", 0);
  }

  void gpsCallback(const rover12_comm::GpsMsg& msg) {

  }

  void imuCallback(const rover12_comm::ImuMsg& msg) {
    sensor_msgs::Imu ros_msg;

    // Timestamp and identify reference frame.
    ros_msg.header.stamp = ros::Time::now();
    ros_msg.header.frame_id = "imu";

    // Set absolute rotation.
    geometry_msgs::Quaternion quat;
    quat.x = msg.data.orient_x;
    quat.y = msg.data.orient_y;
    quat.z = msg.data.orient_z;
    quat.w = msg.data.orient_w;
    ros_msg.orientation = quat;

    // TODO: Come up with reasonable orientation covariance.
    ros_msg.orientation_covariance[0] = 0.0;
    ros_msg.orientation_covariance[1] = 0.0;
    ros_msg.orientation_covariance[2] = 0.0;
    ros_msg.orientation_covariance[3] = 0.0;
    ros_msg.orientation_covariance[4] = 0.0;
    ros_msg.orientation_covariance[5] = 0.0;
    ros_msg.orientation_covariance[6] = 0.0;
    ros_msg.orientation_covariance[7] = 0.0;
    ros_msg.orientation_covariance[8] = 0.0;

    // Set angular velocity.
    geometry_msgs::Vector3 ang_vel;
    ang_vel.x = msg.data.ang_vel_x;
    ang_vel.y = msg.data.ang_vel_y;
    ang_vel.z = msg.data.ang_vel_z;
    ros_msg.angular_velocity = ang_vel;

    // TODO: Come up with reasonable angular velocity covariance.
    ros_msg.angular_velocity_covariance[0] = 0.0;
    ros_msg.angular_velocity_covariance[1] = 0.0;
    ros_msg.angular_velocity_covariance[2] = 0.0;
    ros_msg.angular_velocity_covariance[3] = 0.0;
    ros_msg.angular_velocity_covariance[4] = 0.0;
    ros_msg.angular_velocity_covariance[5] = 0.0;
    ros_msg.angular_velocity_covariance[6] = 0.0;
    ros_msg.angular_velocity_covariance[7] = 0.0;
    ros_msg.angular_velocity_covariance[8] = 0.0;

    // Set linear acceleration.
    geometry_msgs::Vector3 lin_accel;
    lin_accel.x = msg.data.lin_accel_x;
    lin_accel.y = msg.data.lin_accel_y;
    lin_accel.z = msg.data.lin_accel_z;
    ros_msg.linear_acceleration = lin_accel;

    // TODO: Come up with reasonable linear acceleration covariance.
    ros_msg.linear_acceleration_covariance[0] = 0.0;
    ros_msg.linear_acceleration_covariance[1] = 0.0;
    ros_msg.linear_acceleration_covariance[2] = 0.0;
    ros_msg.linear_acceleration_covariance[3] = 0.0;
    ros_msg.linear_acceleration_covariance[4] = 0.0;
    ros_msg.linear_acceleration_covariance[5] = 0.0;
    ros_msg.linear_acceleration_covariance[6] = 0.0;
    ros_msg.linear_acceleration_covariance[7] = 0.0;
    ros_msg.linear_acceleration_covariance[8] = 0.0;

    // Publish!
    imu_pub_.publish(ros_msg);
  }

  void imuCalCallback(const rover12_comm::ImuCalMsg& msg) {
    rover12_drivers::ImuCalibration ros_msg;

    // Timestamp and identify reference frame.
    ros_msg.header.stamp = ros::Time::now();
    ros_msg.header.frame_id = "imu";

    // Populate the calibration status.
    ros_msg.system = msg.data.system;
    ros_msg.gyroscope = msg.data.gyro;
    ros_msg.accelerometer = msg.data.accel;
    ros_msg.magnetometer = msg.data.mag;

    // Publish!
    imu_cal_pub_.publish(ros_msg);
  }

private:
  ros::NodeHandle nh_;
  ros::Publisher imu_pub_;
  ros::Publisher imu_cal_pub_;
};

int main(int argc, char* argv[]) {
  if (argc < 2) {
    ROS_ERROR_STREAM("Usage: " << argv[0] << " [serial devices]");
    return EXIT_FAILURE;
  }

  // Initialize ROS.
  ros::init(argc, argv, "sensor_board_node");

  // Create sensor publisher.
  SensorPublisher sensor_publisher;

  // Add each serial device to the messenger.
  rover12_drivers::Messenger messenger;
  for (int i = 1; i < argc; ++i) {
    messenger.addDevice(argv[i]);
  }

  // Set callbacks on message type.
  messenger.setGpsCallback([&sensor_publisher](const rover12_comm::GpsMsg& msg) {
    sensor_publisher.gpsCallback(msg);
  });
  messenger.setImuCallback([&sensor_publisher](const rover12_comm::ImuMsg& msg) {
    sensor_publisher.imuCallback(msg);
  });
  messenger.setImuCalCallback([&sensor_publisher](const rover12_comm::ImuCalMsg& msg) {
    sensor_publisher.imuCalCallback(msg);
  });

  // Connect to the sensor board.
  messenger.connect(rover12_drivers::Messenger::Board::SENSOR);

  // Spin on the messenger and ROS.
  while (ros::ok()) {
    messenger.spin();
    ros::spinOnce();
  }

  return EXIT_SUCCESS;
};
