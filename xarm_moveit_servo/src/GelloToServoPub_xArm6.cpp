// GelloToServoPub - Gello teleoperation to MoveIt Servo velocity commands
// For keyboard control, use the standalone gello_keyboard_input node
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/bool.hpp>

#include <unordered_map>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <mutex>

using rclcpp::Node;
using control_msgs::msg::JointJog;
using sensor_msgs::msg::JointState;

namespace xarm_moveit_servo
{
class GelloToServoPub : public rclcpp::Node
{
public:
  explicit GelloToServoPub(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("gello_to_servo_vel", options)
  {
    // --- Parameters ---
    declare_parameter<std::vector<std::string>>("follower_joint_names",
      {"joint1","joint2","joint3","joint4","joint5","joint6"});

    declare_parameter<std::string>("leader_topic", "gello/joint_states");
    declare_parameter<std::string>("follower_states_topic", "/joint_states");
    declare_parameter<std::string>("joint_cmd_topic", "servo_server/delta_joint_cmds");
    // Jazzy: start_servo is gone; use switch_command_type to declare JointJog mode (JOINT_JOG=0)
    declare_parameter<std::string>("switch_command_type_srv", "/servo_server/switch_command_type");

    declare_parameter<double>("rate_hz", 200.0);
    declare_parameter<double>("deadband_rad", 1e-4);
    declare_parameter<double>("vel_deadband", 1e-3);
    declare_parameter<double>("kp", 4.0);
    declare_parameter<double>("kd", 0.01);
    declare_parameter<double>("k_ff", 0.6);

    declare_parameter<double>("max_vel_per_joint", 1.0);
    declare_parameter<double>("max_accel_per_joint", 25.0);
    declare_parameter<double>("vel_filter_tau_s", 0.05);

    declare_parameter<double>("leader_timeout_s", 0.25);
    declare_parameter<double>("state_timeout_s", 0.25);
    // Ignore tiny changes in leader joint position between messages (filters jitter before diff)
    declare_parameter<double>("pos_change_threshold_rad", 0.005);

    follower_joint_names_ = get_parameter("follower_joint_names").as_string_array();

    leader_topic_          = get_parameter("leader_topic").as_string();
    follower_states_topic_ = get_parameter("follower_states_topic").as_string();
    joint_cmd_topic_       = get_parameter("joint_cmd_topic").as_string();
    switch_cmd_type_srv_   = get_parameter("switch_command_type_srv").as_string();

    rate_hz_        = get_parameter("rate_hz").as_double();
    deadband_       = get_parameter("deadband_rad").as_double();
    vel_deadband_   = get_parameter("vel_deadband").as_double();
    kp_             = get_parameter("kp").as_double();
    kd_             = get_parameter("kd").as_double();
    k_ff_           = get_parameter("k_ff").as_double();
    vmax_           = get_parameter("max_vel_per_joint").as_double();
    amax_           = get_parameter("max_accel_per_joint").as_double();
    tau_            = get_parameter("vel_filter_tau_s").as_double();
    leader_timeout_ = get_parameter("leader_timeout_s").as_double();
    state_timeout_  = get_parameter("state_timeout_s").as_double();
    pos_change_thresh_rad_ = get_parameter("pos_change_threshold_rad").as_double();

    const size_t N = follower_joint_names_.size();
    follower_pos_.assign(N, 0.0);
    follower_vel_.assign(N, 0.0);
    leader_pos_now_.assign(N, 0.0);
    leader_pos_last_.assign(N, 0.0);
    leader_vel_now_.assign(N, 0.0);
    tmp_err_.assign(N, 0.0);
    cmd_vel_raw_.assign(N, 0.0);
    prev_cmd_vel_.assign(N, 0.0);
    filt_cmd_vel_.assign(N, 0.0);

    // --- I/O ---
    leader_sub_ = create_subscription<JointState>(
      leader_topic_, rclcpp::SensorDataQoS(),
      [this](JointState::SharedPtr msg){ on_leader(std::move(msg)); });

    follower_sub_ = create_subscription<JointState>(
      follower_states_topic_, rclcpp::SensorDataQoS(),
      [this](JointState::SharedPtr msg){ on_follower(std::move(msg)); });
    
    rclcpp::QoS qos_servo_pub(1);
    qos_servo_pub.reliable();
    qos_servo_pub.durability_volatile();
    pub_ = create_publisher<JointJog>(joint_cmd_topic_, qos_servo_pub);

    // Control loop
    const double hz = std::max(20.0, rate_hz_);
    loop_dt_ = 1.0 / hz;
    timer_ = create_wall_timer(
      std::chrono::duration<double>(loop_dt_),
      [this](){ tick(); });

    // Jazzy: tell Servo to expect JointJog commands (JOINT_JOG=0, TWIST=1, POSE=2)
    // Verify with: ros2 interface show moveit_msgs/srv/ServoCommandType
    switch_cmd_type_ = create_client<moveit_msgs::srv::ServoCommandType>(switch_cmd_type_srv_);
    (void)switch_cmd_type_->wait_for_service(std::chrono::seconds(2));
    call_switch_command_type(moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG);

    // Service to pause/resume Gello control (for keyboard override)
    pause_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/set_gello_paused",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
             std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        on_set_paused(req, res);
      });

    // Also provide a simple toggle service
    toggle_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/toggle_gello",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
             std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        std::lock_guard<std::mutex> lock(pause_mutex_);
        gello_paused_ = !gello_paused_;
        if (!gello_paused_) {
          reset_on_unpause();
        }
        res->success = true;
        res->message = gello_paused_ ? "Gello PAUSED (keyboard override)" : "Gello ACTIVE";
        RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
      });

    // Publisher for pause state (so keyboard node can show status)
    gello_paused_pub_ = create_publisher<std_msgs::msg::Bool>("~/gello_paused", 10);

    RCLCPP_INFO(get_logger(),
      "Gello→Servo velocity mode: N=%zu, rate=%.1f Hz, kp=%.3f, kd=%.3f, k_ff=%.3f, vmax=%.2f, amax=%.2f, tau=%.3f, dpos_thr=%.6f",
      N, hz, kp_, kd_, k_ff_, vmax_, amax_, tau_, pos_change_thresh_rad_);
    RCLCPP_INFO(get_logger(),
      "For keyboard twist control, launch with enable_keyboard:=true or run gello_keyboard_input node separately");
    RCLCPP_INFO(get_logger(),
      "Toggle Gello pause: call ~/toggle_gello service or ~/set_gello_paused (SetBool)");
  }

private:
  // Helpers
  static inline double wrap(double d){ return std::atan2(std::sin(d), std::cos(d)); }
  static inline double clamp(double x,double lo,double hi){ return std::max(lo,std::min(hi,x)); }

  void call_switch_command_type(int8_t type)
  {
    if (!switch_cmd_type_ || !switch_cmd_type_->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "switch_command_type service not ready, skipping");
      return;
    }
    auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    req->command_type = type;
    switch_cmd_type_->async_send_request(req,
      [this, type](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture fut) {
        auto res = fut.get();
        if (!res->success)
          RCLCPP_WARN(get_logger(), "switch_command_type(%d) failed: %s", type, res->message.c_str());
      });
  }

  // Handle pause/resume service
  void on_set_paused(const std::shared_ptr<std_srvs::srv::SetBool::Request>& req,
                     std::shared_ptr<std_srvs::srv::SetBool::Response>& res)
  {
    std::lock_guard<std::mutex> lock(pause_mutex_);
    bool was_paused = gello_paused_;
    gello_paused_ = req->data;
    
    // If we're un-pausing, reset velocity state to avoid sudden movement
    if (was_paused && !gello_paused_) {
      reset_on_unpause();
    }
    
    res->success = true;
    res->message = gello_paused_ ? "Gello PAUSED (keyboard override)" : "Gello ACTIVE";
    RCLCPP_INFO(get_logger(), "%s", res->message.c_str());
  }

  // Reset velocity state when un-pausing to avoid sudden jumps
  void reset_on_unpause()
  {
    // Reset velocity tracking - start fresh
    std::fill(prev_cmd_vel_.begin(), prev_cmd_vel_.end(), 0.0);
    std::fill(filt_cmd_vel_.begin(), filt_cmd_vel_.end(), 0.0);
    std::fill(leader_vel_now_.begin(), leader_vel_now_.end(), 0.0);
    
    // Reset the "last" leader position to current, so we don't get a velocity spike
    leader_pos_last_ = leader_pos_now_;
    
    // Mark that we need a fresh velocity estimate
    have_leader_last_ = false;

    // Re-declare JointJog mode: keyboard may have switched Servo to TWIST while it had control
    call_switch_command_type(moveit_msgs::srv::ServoCommandType::Request::JOINT_JOG);

    RCLCPP_INFO(get_logger(), "Gello state reset - velocity tracking restarted");
  }

  // Reorder into follower order. If leader msg has names, use them.
  // If no names but positions length matches, assume same order.
  bool reorder_into_follower(const JointState& js, std::vector<double>& out_pos) const
  {
    const size_t N = follower_joint_names_.size();
    out_pos.resize(N);

    if (!js.name.empty()) {
      std::unordered_map<std::string, size_t> idx;
      idx.reserve(js.name.size());
      for (size_t i=0;i<js.name.size();++i) idx[js.name[i]] = i;
      for (size_t i=0;i<N;++i) {
        auto it = idx.find(follower_joint_names_[i]);
        if (it == idx.end() || it->second >= js.position.size()) return false;
        out_pos[i] = js.position[it->second];
      }
      return true;
    } else if (js.position.size() == N) {
      std::copy(js.position.begin(), js.position.end(), out_pos.begin());
      return true;
    }
    return false;
  }

  void on_follower(const JointState::SharedPtr& msg)
  {
    last_follower_stamp_ = now();

    // Positions (reordered) and velocities (if provided, also reorder by name; else zeros)
    if (!reorder_into_follower(*msg, follower_pos_)) {
      have_follower_ = false;
      return;
    }

    // Velocities
    follower_vel_.assign(follower_vel_.size(), 0.0);
    if (!msg->name.empty() && msg->velocity.size() == msg->name.size()) {
      std::unordered_map<std::string, size_t> idx;
      idx.reserve(msg->name.size());
      for (size_t i=0;i<msg->name.size();++i) idx[msg->name[i]] = i;
      for (size_t i=0;i<follower_joint_names_.size();++i) {
        auto it = idx.find(follower_joint_names_[i]);
        if (it != idx.end()) follower_vel_[i] = msg->velocity[it->second];
      }
    } else if (msg->name.empty() && msg->velocity.size() == follower_vel_.size()) {
      std::copy(msg->velocity.begin(), msg->velocity.end(), follower_vel_.begin());
    }

    have_follower_ = true;
  }

  void on_leader(const JointState::SharedPtr& msg)
  {
    // Expect Gello already aligned; reorder into follower order
    if (!reorder_into_follower(*msg, leader_pos_now_)) {
      have_leader_ = false;
      return;
    }

    // Get current ROS time from the node clock
    const rclcpp::Time t_now = now();

    double dt = 0.0;
    if (have_leader_last_) {
      // Only subtract when both stamps come from the same clock
      dt = (t_now - last_leader_stamp_).seconds();
    }
    last_leader_stamp_ = t_now;

    // Ignore tiny position steps on leader to reduce jitter before differentiating
    if (have_leader_last_ && pos_change_thresh_rad_ > 0.0) {
      for (size_t i=0;i<leader_pos_now_.size();++i) {
        const double d = wrap(leader_pos_now_[i] - leader_pos_last_[i]);
        if (std::fabs(d) < pos_change_thresh_rad_) {
          leader_pos_now_[i] = leader_pos_last_[i];
        }
      }
    }

    // Finite-difference leader velocity with unwrap
    if (have_leader_last_ && dt > 1e-4 && dt < 1.0) {
      for (size_t i=0;i<leader_pos_now_.size();++i) {
        const double d = wrap(leader_pos_now_[i] - leader_pos_last_[i]);
        leader_vel_now_[i] = d / dt;
      }
    } else {
      std::fill(leader_vel_now_.begin(), leader_vel_now_.end(), 0.0);
    }

    leader_pos_last_ = leader_pos_now_;
    have_leader_ = true;
    have_leader_last_ = true;
  }

  void tick()
  {
    // Publish pause state periodically
    {
      std_msgs::msg::Bool pause_msg;
      pause_msg.data = gello_paused_;
      gello_paused_pub_->publish(pause_msg);
    }

    if (!have_follower_ || !have_leader_) return;

    // Watchdogs
    if ((now() - last_leader_stamp_).seconds()   > leader_timeout_) return;
    if ((now() - last_follower_stamp_).seconds() > state_timeout_)  return;

    // If paused, don't send any commands (keyboard has full control)
    // But we keep updating state so un-pause is smooth
    {
      std::lock_guard<std::mutex> lock(pause_mutex_);
      if (gello_paused_) {
        // Keep velocity state zeroed while paused so we don't accumulate
        std::fill(prev_cmd_vel_.begin(), prev_cmd_vel_.end(), 0.0);
        std::fill(filt_cmd_vel_.begin(), filt_cmd_vel_.end(), 0.0);
        return;
      }
    }

    const size_t N = follower_joint_names_.size();

    // 1) error
    bool all_small = true;
    for (size_t i=0;i<N;++i) {
      double e = wrap(leader_pos_now_[i] - follower_pos_[i]);
      if (std::fabs(e) < deadband_) e = 0.0; else all_small = false;
      tmp_err_[i] = e;
    }

    // 2) velocity command = FF + P - D
    for (size_t i=0;i<N;++i) {
      const double vff = (std::fabs(leader_vel_now_[i]) < vel_deadband_) ? 0.0 : (k_ff_ * leader_vel_now_[i]);
      const double vp  = kp_ * tmp_err_[i];
      const double vd  = -kd_ * follower_vel_[i];
      cmd_vel_raw_[i]  = vff + vp + vd;
    }

    // 3) hard vel cap
    for (double& v : cmd_vel_raw_) v = clamp(v, -vmax_, +vmax_);

    // 4) slew-rate (accel) limit
    for (size_t i=0;i<N;++i) {
      const double dv = cmd_vel_raw_[i] - prev_cmd_vel_[i];
      const double dv_max = amax_ * loop_dt_;
      prev_cmd_vel_[i] += clamp(dv, -dv_max, +dv_max);
    }

    // 5) EMA smoothing
    const double alpha = (tau_ <= 1e-6) ? 1.0 : (loop_dt_ / (tau_ + loop_dt_));
    for (size_t i=0;i<N;++i) {
      filt_cmd_vel_[i] += alpha * (prev_cmd_vel_[i] - filt_cmd_vel_[i]);
    }

    // 6) skip spam if tiny
    bool all_zeroish = true;
    for (double v : filt_cmd_vel_) if (std::fabs(v) > 1e-6) { all_zeroish = false; break; }
    if (all_small && all_zeroish) return;

    // 7) publish velocities only
    JointJog jj;
    jj.header.stamp = now();
    jj.joint_names  = follower_joint_names_;
    jj.velocities   = filt_cmd_vel_;
    jj.displacements.clear();   // IMPORTANT: empty in velocity mode
    pub_->publish(jj);
  }

  // Params
  std::vector<std::string> follower_joint_names_;
  std::string leader_topic_, follower_states_topic_, joint_cmd_topic_, switch_cmd_type_srv_;
  double rate_hz_{200.0}, loop_dt_{0.005};
  double deadband_{1e-4}, vel_deadband_{1e-3};
  double kp_{4.0}, kd_{0.0}, k_ff_{1.0};
  double vmax_{1.0}, amax_{10.0}, tau_{0.02};
  double leader_timeout_{0.25}, state_timeout_{0.25};
  double pos_change_thresh_rad_{0.0};

  // State
  std::vector<double> follower_pos_, follower_vel_;
  std::vector<double> leader_pos_now_, leader_pos_last_, leader_vel_now_;
  std::vector<double> tmp_err_, cmd_vel_raw_, prev_cmd_vel_, filt_cmd_vel_;
  rclcpp::Time last_leader_stamp_, last_follower_stamp_;
  bool have_follower_{false}, have_leader_{false}, have_leader_last_{false};

  // ROS
  rclcpp::Subscription<JointState>::SharedPtr leader_sub_, follower_sub_;
  rclcpp::Publisher<JointJog>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_cmd_type_;

  // Pause control (for keyboard override)
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr pause_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr toggle_srv_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gello_paused_pub_;
  std::mutex pause_mutex_;
  bool gello_paused_{false};
};
} // namespace xarm_moveit_servo

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(xarm_moveit_servo::GelloToServoPub)
