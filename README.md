# Rover 12

An autonomous 1/10th scale R/C car built by Maria Pikusova and Bob Somers.

## Documentation

### Nodes

* `/joint_state_publisher` (joint\_state\_publisher/joint\_state\_publisher):
  Listens on individual joint topics under `/joints` and  publishes a single
  sensor\_msgs/JointState which contains the state of all joints in the robot
  (based on the URDF).
* `/robot_state_publisher` (robot\_state\_publisher/state\_publisher): Listens
  to the combined joint states of the robot on `/joint_states`, runs forward
  kinematics based on the URDF, and publishes the results to the TF2 tree.

### Topics

* `/joint_states` (sensor\_msgs/JointState): A combined, consistent JointState
  for all joints in the robot, based on the URDF. Published by the
  `joint_state_publisher`.
* `/joints`: Namespace for controlling individual joint states.
  * `/fl_steer` (sensor\_msgs/JointState): Steering rotation of the front left kingpin.
  * `/fr_steer` (sensor\_msgs/JointState): Steering rotation of the front right kingpin.
  * `/fl_wheel` (sensor\_msgs/JointState): Rotation of the front left wheel.
  * `/fr_wheel` (sensor\_msgs/JointState): Rotation of the front right wheel.
  * `/rl_wheel` (sensor\_msgs/JointState): Rotation of the rear left wheel.
  * `/rr_wheel` (sensor\_msgs/JointState): Rotation of the rear right wheel.

### TF2 Frames
* `odom`: World-fixed frame in which `base_link` moves smoothly over the short
  term, but accumulates drift error over the long term. The origin of this frame
  is the startup location of the robot.
  * `base_link`: Fixed to robot, parent of all other frames attached to the
    robot. Origin is at the rear axle (X forward, Y out the driver's side, Z
    up).
    * `imu`: Origin at the sensing center of the IMU, oriented according to the
      IMUs local coordinate frame.
    * `gps`: Origen at the sensing center of the GPS antenna, oriented in the
      same direction as `base_link`.
    * `body`: Origin at the approximate center of mass of the body shell,
      primarily used for drawing the robot model at a sensible location in rviz.
    * `rl_wheel`: Origin in the center of the rear left wheel. Rotates about
      its Y axis as the wheel rotates. Located half the track distance from
      `base_link` along its Y axis.
    * `rr_wheel`: Origin in the center of the rear right wheel. Rotates about
      its Y axis as the wheel rotates. Located half the track distance from
      `base_link` along its Y axis.
    * `f_axle`: Origin in the center of the front axle, oriented in the same
      way as `base_link`. The distance between this frame and `base_link` is the
      wheelbase distance along their colinear X axes.
      * `fl_kingpin`: Origin in the steering center of the front left wheel,
        located half the track distance from `f_axle` along its Y axis. Rotates
        about its Z axis.
        * `fl_wheel`: Origin in the center of the front left wheel, orientation
          accounting for the rotation of `fl_kingpin`. Rotates about its Y axis
          as the wheel rotates.
      * `fr_kingpin`: Origin in the steering center of the front right wheel,
        located half the track distance from `f_axle` along its Y axis. Rotates
        about its Z axis.
        * `fr_wheel`: Origin in the center of the front right wheel, orientation
          accounting for the rotation of `fr_kingpin`. Rotates about its Y axis
          as the wheel rotates.
