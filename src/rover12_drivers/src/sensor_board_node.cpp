#include <arpa/inet.h>

#include <cstdlib>
#include <string>

#include <ros/ros.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <rover12_comm/rover12_comm.h>
#include <rover12_drivers/GpsStatus.h>
#include <rover12_drivers/ImuStatus.h>
#include <rover12_drivers/messenger.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>

namespace {

double extractInt32ToDouble(const uint8_t* buf, double scale) {
  uint32_t tmp = *reinterpret_cast<const uint32_t*>(buf);
  uint32_t swapped = ntohl(tmp);
  int32_t value = *reinterpret_cast<int32_t*>(&swapped);
  return value * scale;
}

double extractUInt32ToDouble(const uint8_t* buf, double scale) {
  uint32_t tmp = *reinterpret_cast<const uint32_t*>(buf);
  uint32_t swapped = ntohl(tmp);
  return swapped * scale;
}

} // namespace

class SensorPublisher {
public:
  explicit SensorPublisher(ros::NodeHandle& nh)
    : nh_(nh) {
    gps_pub_ = nh_.advertise<sensor_msgs::NavSatFix>("/sensors/gps", 0);
    gps_status_pub_ = nh_.advertise<rover12_drivers::GpsStatus>("/sensors/gps_status", 0);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/sensors/imu", 0);
    imu_cal_pub_ = nh_.advertise<rover12_drivers::ImuStatus>("/sensors/imu_status", 0);

    // Absolute orientation covaraince.
    const double roll_cov = 0.001;
    const double pitch_cov = 0.001;
    const double yaw_cov = 0.001;
    imu_msg_.orientation_covariance[0] = roll_cov;
    imu_msg_.orientation_covariance[1] = 0.0;
    imu_msg_.orientation_covariance[2] = 0.0;
    imu_msg_.orientation_covariance[3] = 0.0;
    imu_msg_.orientation_covariance[4] = pitch_cov;
    imu_msg_.orientation_covariance[5] = 0.0;
    imu_msg_.orientation_covariance[6] = 0.0;
    imu_msg_.orientation_covariance[7] = 0.0;
    imu_msg_.orientation_covariance[8] = yaw_cov;

    // Angular velocity covariance.
    const double xv_cov = -1.0;
    const double yv_cov = -1.0;
    const double zv_cov = -1.0;
    imu_msg_.angular_velocity_covariance[0] = xv_cov;
    imu_msg_.angular_velocity_covariance[1] = 0.0;
    imu_msg_.angular_velocity_covariance[2] = 0.0;
    imu_msg_.angular_velocity_covariance[3] = 0.0;
    imu_msg_.angular_velocity_covariance[4] = yv_cov;
    imu_msg_.angular_velocity_covariance[5] = 0.0;
    imu_msg_.angular_velocity_covariance[6] = 0.0;
    imu_msg_.angular_velocity_covariance[7] = 0.0;
    imu_msg_.angular_velocity_covariance[8] = zv_cov;

    // Linear acceleration covariance.
    const double xa_cov = -1.0;
    const double ya_cov = -1.0;
    const double za_cov = -1.0;
    imu_msg_.linear_acceleration_covariance[0] = xa_cov;
    imu_msg_.linear_acceleration_covariance[1] = 0.0;
    imu_msg_.linear_acceleration_covariance[2] = 0.0;
    imu_msg_.linear_acceleration_covariance[3] = 0.0;
    imu_msg_.linear_acceleration_covariance[4] = ya_cov;
    imu_msg_.linear_acceleration_covariance[5] = 0.0;
    imu_msg_.linear_acceleration_covariance[6] = 0.0;
    imu_msg_.linear_acceleration_covariance[7] = 0.0;
    imu_msg_.linear_acceleration_covariance[8] = za_cov;

    // TODO: Come up with reasonable position covariance. Maybe try to compute
    // based on DOP data in the GPS message?
    gps_msg_.position_covariance[0] = 0.0;
    gps_msg_.position_covariance[1] = 0.0;
    gps_msg_.position_covariance[2] = 0.0;
    gps_msg_.position_covariance[3] = 0.0;
    gps_msg_.position_covariance[4] = 0.0;
    gps_msg_.position_covariance[5] = 0.0;
    gps_msg_.position_covariance[6] = 0.0;
    gps_msg_.position_covariance[7] = 0.0;
    gps_msg_.position_covariance[8] = 0.0;

    // TODO: Change this to an appropriate type.
    gps_msg_.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
  }

  void gpsCallback(const rover12_comm::GpsMsg& msg) {
    uint8_t quality = msg.data.bytes[1];
    uint8_t num_sats = msg.data.bytes[2];

    // TODO: This probably isn't correct. 2D, 3D, 3D + DGPS does not directly
    // map to FIX, SBAS_FIX, GBAS_FIX.
    sensor_msgs::NavSatStatus status_msg;
    status_msg.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    if (quality == 1) {
      status_msg.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    } else if (quality == 2) {
      status_msg.status = sensor_msgs::NavSatStatus::STATUS_SBAS_FIX;
    } else if (quality == 3) {
      status_msg.status = sensor_msgs::NavSatStatus::STATUS_GBAS_FIX;
    }

    // Venus only does GPS I think.
    status_msg.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

    double lat = extractInt32ToDouble(&msg.data.bytes[9], 1.0e-7);
    double lng = extractInt32ToDouble(&msg.data.bytes[13], 1.0e-7);
    double alt = extractUInt32ToDouble(&msg.data.bytes[17], 0.01);

    // Timestamp and identify reference frame.
    gps_msg_.header.stamp = ros::Time::now();
    gps_msg_.header.frame_id = "gps";

    // Fix status.
    gps_msg_.status = status_msg;

    // Position.
    gps_msg_.latitude = lat;
    gps_msg_.longitude = lng;
    gps_msg_.altitude = alt;

    // Publish!
    gps_pub_.publish(gps_msg_);

    // Publish extra status information.
    rover12_drivers::GpsStatus extra_status;
    extra_status.header.stamp = ros::Time::now();
    extra_status.header.frame_id = "gps";
    extra_status.quality = quality;
    extra_status.num_sats = num_sats;
    gps_status_pub_.publish(extra_status);
  }

  void imuCallback(const rover12_comm::ImuMsg& msg) {
    // Timestamp and identify reference frame.
    imu_msg_.header.stamp = ros::Time::now();
    imu_msg_.header.frame_id = "imu";

    // Set absolute rotation.
    geometry_msgs::Quaternion quat;
    quat.x = msg.data.orient_x;
    quat.y = msg.data.orient_y;
    quat.z = msg.data.orient_z;
    quat.w = msg.data.orient_w;
    imu_msg_.orientation = quat;

    // Set angular velocity.
    geometry_msgs::Vector3 ang_vel;
    ang_vel.x = msg.data.ang_vel_x;
    ang_vel.y = msg.data.ang_vel_y;
    ang_vel.z = msg.data.ang_vel_z;
    imu_msg_.angular_velocity = ang_vel;

    // Set linear acceleration.
    geometry_msgs::Vector3 lin_accel;
    lin_accel.x = msg.data.lin_accel_x;
    lin_accel.y = msg.data.lin_accel_y;
    lin_accel.z = msg.data.lin_accel_z;
    imu_msg_.linear_acceleration = lin_accel;

    // Publish!
    imu_pub_.publish(imu_msg_);
  }

  void imuCalCallback(const rover12_comm::ImuCalMsg& msg) {
    rover12_drivers::ImuStatus ros_msg;

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
  ros::NodeHandle& nh_;
  ros::Publisher gps_pub_;
  ros::Publisher gps_status_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher imu_cal_pub_;
  sensor_msgs::Imu imu_msg_;
  sensor_msgs::NavSatFix gps_msg_;
};

int main(int argc, char* argv[]) {
  if (argc < 2) {
    ROS_ERROR_STREAM("Usage: " << argv[0] << " [serial devices]");
    return EXIT_FAILURE;
  }

  // Initialize ROS.
  ros::init(argc, argv, "sensor_board_node");
  ros::NodeHandle nh;

  // Create sensor publisher.
  SensorPublisher sensor_publisher(nh);

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
