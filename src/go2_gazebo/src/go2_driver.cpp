// go2_driver.cpp -- Unitree Go2 "patrol mode" driver.
//
// Subscribes to /cmd_vel (Twist, the same one the bridge forwards to the gz
// VelocityControl system), integrates it into /odom + odom->base TF, and drives
// a stylised trot gait on the 12 leg joints:
//   * /joint_states         (sensor_msgs/JointState)  -- for robot_state_publisher
//   * /go2/joint_trajectory (trajectory_msgs/JointTrajectory)
//     -- bridged to gz /model/go2/joint_trajectory, followed by the
//        JointTrajectoryController system so the *simulated* legs animate.
//
// The gait is a pure kinematic keyframe animation (this sim has zero gravity and
// no ground contact by design). Leg-lift amounts were validated against the real
// go2.urdf forward kinematics so the feet never drop into the ground plane.

#include <algorithm>
#include <array>
#include <cmath>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace go2
{

namespace
{
// Order must match the JointTrajectoryController's <joint_name> list, which the
// converter emits in go2.urdf document order (FL, FR, RL, RR, hip->calf each).
constexpr std::array<const char *, 12> kJointOrder = {
  "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
  "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
  "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
  "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
};
// Diagonal-trot phase offsets (fraction of stride) for FL, FR, RL, RR.
constexpr std::array<double, 4> kTrotPhase = {0.0, 0.5, 0.5, 0.0};
constexpr double kTwoPi = 2.0 * M_PI;
}  // namespace

class Go2Driver : public rclcpp::Node
{
public:
  Go2Driver() : Node("go2_driver")
  {
    declare_params();

    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(1).best_effort(),
      [this](geometry_msgs::msg::Twist::ConstSharedPtr m) { cmd_cb(m); });
    js_pub_ = create_publisher<sensor_msgs::msg::JointState>(js_topic_, 10);
    traj_pub_ =
      create_publisher<trajectory_msgs::msg::JointTrajectory>(traj_topic_, 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(1000 / rate_hz_), [this]() { tick(); });

    // Publish an initial standing pose so the controller holds the robot up
    // even before any /cmd_vel arrives.
    std::array<double, 12> standing{};
    fill_standing(standing);
    publish_legs(standing);
  }

private:
  void cmd_cb(geometry_msgs::msg::Twist::ConstSharedPtr m)
  {
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    cmd_ = *m;
  }

  void fill_standing(std::array<double, 12> & q) const
  {
    for (size_t i = 0; i < q.size(); ++i) {
      const std::string n = kJointOrder[i];
      q[i] = (n.find("thigh") != std::string::npos) ? 0.9 :
             (n.find("calf") != std::string::npos) ? -1.8 : 0.0;
    }
  }

  void tick()
  {
    const double dt = 1.0 / rate_hz_;

    // ----- current command (locked) -----
    double vx, vy, wz;
    {
      std::lock_guard<std::mutex> lock(cmd_mutex_);
      vx = cmd_.linear.x;
      vy = cmd_.linear.y;
      wz = cmd_.angular.z;
    }

    // ----- odometry integration in the odom frame (planar) -----
    yaw_ += wz * dt;
    while (yaw_ > M_PI) { yaw_ -= kTwoPi; }
    while (yaw_ < -M_PI) { yaw_ += kTwoPi; }
    const double cy = std::cos(yaw_), sy = std::sin(yaw_);
    x_ += (vx * cy - vy * sy) * dt;
    y_ += (vx * sy + vy * cy) * dt;
    publish_odom();

    // ----- gait -----
    // motion metric: linear dominates, rotation adds a little "step in place"
    const double speed = std::hypot(vx, vy);
    const double metric =
      std::min(1.0, (speed + turn_weight_ * std::abs(wz)) / speed_norm_);
    if (metric > 0.03) {
      phase_ += dt * kTwoPi * (gait_freq_ * std::max(0.1, metric));
      phase_ = std::fmod(phase_, kTwoPi);
    } else {
      phase_ = 0.0;   // parked -> pure standing pose
    }
    const double amp = metric;  // lift scales with commanded speed

    // ----- joint keyframes -----
    std::array<double, 12> q{};
    for (int leg = 0; leg < 4; ++leg) {
      // stride fraction, diagonal pairs (FL+RR / FR+RL) share a phase
      const double s = std::fmod(phase_ / kTwoPi + kTrotPhase[leg], 1.0);
      const double swing_duty = 0.5;
      double lift = 0.0;
      if (s < swing_duty) {
        const double u = s / swing_duty;       // 0..1 within the swing window
        lift = 0.5 - 0.5 * std::cos(kTwoPi * u);  // smooth 0->1->0
      }
      // validated foot-clearance envelope: thigh +0.28, calf -0.45
      const double thigh = 0.9 + 0.28 * lift * amp;
      const double calf = -1.8 - 0.45 * lift * amp;
      const size_t i = 3u * static_cast<size_t>(leg);  // 0,3,6,9
      q[i] = 0.0;         // hip (roll) stays straight
      q[i + 1] = thigh;
      q[i + 2] = calf;
    }
    publish_legs(q);
  }

  void publish_odom()
  {
    const auto stamp = now();
    // z-axis quaternion from yaw (roll = pitch = 0 for the planar model)
    const double hz = 0.5 * yaw_;
    const double qz = std::sin(hz), qw = std::cos(hz);

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = odom_frame_;
    t.child_frame_id = base_frame_;
    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = qz;
    t.transform.rotation.w = qw;
    tf_broadcaster_->sendTransform(t);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = qz;
    odom.pose.pose.orientation.w = qw;
    odom.twist.twist.linear.x = cmd_.linear.x;
    odom.twist.twist.linear.y = cmd_.linear.y;
    odom.twist.twist.angular.z = cmd_.angular.z;
    odom_pub_->publish(odom);
  }

  void publish_legs(const std::array<double, 12> & q)
  {
    const auto stamp = now();

    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.name.assign(kJointOrder.begin(), kJointOrder.end());
    js.position.assign(q.begin(), q.end());
    js.velocity.resize(12, 0.0);
    js.effort.resize(12, 0.0);
    js_pub_->publish(js);

    trajectory_msgs::msg::JointTrajectory traj;
    traj.header.stamp = stamp;
    traj.header.frame_id = "";
    traj.joint_names.assign(kJointOrder.begin(), kJointOrder.end());
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.assign(q.begin(), q.end());
    pt.velocities.assign(12, 0.0);
    pt.time_from_start = rclcpp::Duration::from_seconds(1.0 / rate_hz_);
    traj.points.push_back(pt);
    traj_pub_->publish(traj);
  }

  void declare_params()
  {
    rate_hz_ = declare_parameter("rate_hz", 50);
    cmd_vel_topic_ = declare_parameter("cmd_vel_topic", "/cmd_vel");
    odom_topic_ = declare_parameter("odom_topic", "/odom");
    js_topic_ = declare_parameter("joint_states_topic", "/joint_states");
    traj_topic_ = declare_parameter("trajectory_topic", "/go2/joint_trajectory");
    odom_frame_ = declare_parameter("odom_frame", "odom");
    base_frame_ = declare_parameter("base_frame", "base");
    gait_freq_ = declare_parameter("gait_freq", 1.6);
    speed_norm_ = declare_parameter("speed_norm", 0.4);
    turn_weight_ = declare_parameter("turn_weight", 0.3);
  }

  // ---- state ----
  int rate_hz_{50};
  std::string cmd_vel_topic_, odom_topic_, js_topic_, traj_topic_;
  std::string odom_frame_, base_frame_;
  double gait_freq_{1.6}, speed_norm_{0.4}, turn_weight_{0.3};

  std::mutex cmd_mutex_;
  geometry_msgs::msg::Twist cmd_;

  double x_{0.0}, y_{0.0}, yaw_{0.0};
  double phase_{0.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

}  // namespace go2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<go2::Go2Driver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
