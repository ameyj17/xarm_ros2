// Standalone keyboard twist control node for use alongside Gello teleoperation
// Based on xarm_keyboard_input.cpp but with WASD/QE/ZX keymap for EEF frame control
// Press SPACE to toggle Gello pause (keyboard-only mode)

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit_msgs/srv/servo_command_type.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/bool.hpp>

// Key codes for WASD + QE + ZX control scheme
#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_Z 0x7A
#define KEYCODE_X 0x78
#define KEYCODE_ESC 0x1B
#define KEYCODE_SPACE 0x20

// Keyboard reader class (matches official implementation)
class KeyboardReader
{
public:
    KeyboardReader() : k_fd_(0)
    {
        tcgetattr(k_fd_, &k_old_termios_);
        struct termios k_termios;
        memcpy(&k_termios, &k_old_termios_, sizeof(struct termios));
        k_termios.c_lflag &= ~(ICANON | ECHO);
        // Setting a new line, then end of file
        k_termios.c_cc[VEOL] = 1;
        k_termios.c_cc[VEOF] = 2;
        tcsetattr(k_fd_, TCSANOW, &k_termios);
    }

    void readOne(char *c)
    {
        int rc = read(k_fd_, c, 1);
        if (rc < 0) {
            throw std::runtime_error("keyboard read failed");
        }
    }

    void shutdown()
    {
        tcsetattr(k_fd_, TCSANOW, &k_old_termios_);
    }

private:
    int k_fd_;
    struct termios k_old_termios_;
};

KeyboardReader keyboard_reader_;

class GelloKeyboardServoPub
{
public:
    GelloKeyboardServoPub(rclcpp::Node::SharedPtr& node)
    : twist_cmd_topic_("/servo_server/delta_twist_cmds"),
      twist_frame_("link_eef"),
      twist_linear_speed_(0.05),   // Fine adjustment: 5cm/s
      twist_angular_speed_(0.1),   // Fine adjustment: ~6 deg/s
      ros_queue_size_(10),
      gello_paused_(false)
    {
        node_ = node;

        // Declare and get parameters
        _declare_or_get_param<std::string>(twist_cmd_topic_, "twist_cmd_topic", twist_cmd_topic_);
        _declare_or_get_param<std::string>(twist_frame_, "twist_frame", twist_frame_);
        _declare_or_get_param<double>(twist_linear_speed_, "twist_linear_speed", twist_linear_speed_);
        _declare_or_get_param<double>(twist_angular_speed_, "twist_angular_speed", twist_angular_speed_);
        _declare_or_get_param<int>(ros_queue_size_, "ros_queue_size", ros_queue_size_);
        _declare_or_get_param<std::string>(gello_node_name_, "gello_node_name", "gello_to_servo_node");

        // Setup publisher
        twist_pub_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
            twist_cmd_topic_, ros_queue_size_);

        // Jazzy: start_servo is gone; switch_command_type is called on mode change, not startup
        switch_cmd_type_client_ = node_->create_client<moveit_msgs::srv::ServoCommandType>(
            "/servo_server/switch_command_type");
        (void)switch_cmd_type_client_->wait_for_service(std::chrono::seconds(1));

        // Create service client to toggle Gello pause
        std::string toggle_service = "/" + gello_node_name_ + "/toggle_gello";
        gello_toggle_client_ = node_->create_client<std_srvs::srv::Trigger>(toggle_service);
        
        // Subscribe to Gello pause state
        std::string pause_topic = "/" + gello_node_name_ + "/gello_paused";
        gello_paused_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            pause_topic, 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                gello_paused_ = msg->data;
            });

        RCLCPP_INFO(node_->get_logger(),
            "GelloKeyboardServoPub initialized: topic=%s, frame=%s, linear=%.2f m/s, angular=%.2f rad/s",
            twist_cmd_topic_.c_str(), twist_frame_.c_str(), twist_linear_speed_, twist_angular_speed_);
        RCLCPP_INFO(node_->get_logger(),
            "Gello toggle service: %s", toggle_service.c_str());
    }

private:
    template <typename T>
    void _declare_or_get_param(T& output_value, const std::string& param_name, const T default_value)
    {
        try {
            if (node_->has_parameter(param_name)) {
                node_->get_parameter<T>(param_name, output_value);
            } else {
                output_value = node_->declare_parameter<T>(param_name, default_value);
            }
        } catch (const rclcpp::exceptions::InvalidParameterTypeException& e) {
            RCLCPP_WARN_STREAM(node_->get_logger(), "InvalidParameterTypeException(" << param_name << "): " << e.what());
            RCLCPP_ERROR_STREAM(node_->get_logger(), "Error getting parameter '" << param_name << "', check parameter type in YAML file");
            throw e;
        }
        RCLCPP_INFO_STREAM(node_->get_logger(), "Found parameter - " << param_name << ": " << output_value);
    }

    void switchServoCommandType(int8_t type)
    {
        if (!switch_cmd_type_client_ || !switch_cmd_type_client_->service_is_ready()) {
            RCLCPP_WARN(node_->get_logger(), "switch_command_type service not ready");
            return;
        }
        auto req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
        req->command_type = type;
        switch_cmd_type_client_->async_send_request(req,
            [this, type](rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedFuture fut) {
                auto res = fut.get();
                if (!res->success)
                    RCLCPP_WARN(node_->get_logger(), "switch_command_type(%d) failed: %s",
                                type, res->message.c_str());
            });
    }

    void toggleGelloPause()
    {
        if (!gello_toggle_client_->service_is_ready()) {
            RCLCPP_WARN(node_->get_logger(), "Gello toggle service not available");
            return;
        }

        // Capture current state: if gello is NOT paused, we're about to pause it (enter keyboard mode)
        bool entering_keyboard_mode = !gello_paused_;
        if (entering_keyboard_mode) {
            // Switch Servo to TWIST before publishing twist commands (TWIST=1)
            switchServoCommandType(moveit_msgs::srv::ServoCommandType::Request::TWIST);
        }
        // Gello node handles switch back to JOINT_JOG when it receives the unpause

        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = gello_toggle_client_->async_send_request(request,
            [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response) {
                auto result = response.get();
                if (result->success) {
                    RCLCPP_INFO(node_->get_logger(), "Gello toggle: %s", result->message.c_str());
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Gello toggle failed: %s", result->message.c_str());
                }
            });
    }

    void printStatus()
    {
        if (gello_paused_) {
            printf("\r\033[1;32m[KEYBOARD MODE]\033[0m - WASD/QE/ZX active, Gello disabled. Press SPACE to switch.   ");
        } else {
            printf("\r\033[1;33m[GELLO MODE]\033[0m    - Gello active, keyboard disabled. Press SPACE to switch.   ");
        }
        fflush(stdout);
    }

public:
    void spin()
    {
        while (rclcpp::ok()) {
            rclcpp::spin_some(node_);
        }
    }

    void keyLoop()
    {
        char c;
        bool publish_twist = false;

        std::thread{std::bind(&GelloKeyboardServoPub::spin, this)}.detach();

        puts("========================================");
        puts("  Gello + Keyboard Teleop Control");
        puts("========================================");
        puts("  SPACE : Switch between GELLO and KEYBOARD mode");
        puts("----------------------------------------");
        puts("Keyboard controls (only in KEYBOARD mode):");
        puts("  W/S : +X / -X (forward/backward)");
        puts("  A/D : +Y / -Y (left/right)");
        puts("  Q/E : +Z / -Z (up/down)");
        puts("  Z/X : +RotZ / -RotZ (yaw CCW/CW)");
        puts("----------------------------------------");
        puts("  ESC or Ctrl+C : Quit (returns to Gello mode)");
        puts("========================================");
        puts("");
        
        printStatus();
        puts("");

        for (;;) {
            try {
                keyboard_reader_.readOne(&c);
            } catch (const std::runtime_error&) {
                perror("read():");
                return;
            }

            RCLCPP_DEBUG(node_->get_logger(), "Key pressed: 0x%02X ('%c')", c, c);

            auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
            twist_msg->header.stamp = node_->now();
            twist_msg->header.frame_id = twist_frame_;

            publish_twist = true;

            switch (c) {
                case KEYCODE_W:  // Forward (+X in EEF frame)
                    if (!gello_paused_) { publish_twist = false; break; }
                    RCLCPP_DEBUG(node_->get_logger(), "W: +X");
                    twist_msg->twist.linear.x = twist_linear_speed_;
                    break;
                case KEYCODE_S:  // Backward (-X in EEF frame)
                    if (!gello_paused_) { publish_twist = false; break; }
                    RCLCPP_DEBUG(node_->get_logger(), "S: -X");
                    twist_msg->twist.linear.x = -twist_linear_speed_;
                    break;
                case KEYCODE_A:  // Left (+Y in EEF frame)
                    if (!gello_paused_) { publish_twist = false; break; }
                    RCLCPP_DEBUG(node_->get_logger(), "A: +Y");
                    twist_msg->twist.linear.y = twist_linear_speed_;
                    break;
                case KEYCODE_D:  // Right (-Y in EEF frame)
                    if (!gello_paused_) { publish_twist = false; break; }
                    RCLCPP_DEBUG(node_->get_logger(), "D: -Y");
                    twist_msg->twist.linear.y = -twist_linear_speed_;
                    break;
                case KEYCODE_Q:  // Up (+Z in EEF frame)
                    if (!gello_paused_) { publish_twist = false; break; }
                    RCLCPP_DEBUG(node_->get_logger(), "Q: +Z");
                    twist_msg->twist.linear.z = twist_linear_speed_;
                    break;
                case KEYCODE_E:  // Down (-Z in EEF frame)
                    if (!gello_paused_) { publish_twist = false; break; }
                    RCLCPP_DEBUG(node_->get_logger(), "E: -Z");
                    twist_msg->twist.linear.z = -twist_linear_speed_;
                    break;
                case KEYCODE_Z:  // Rotate CCW around Z
                    if (!gello_paused_) { publish_twist = false; break; }
                    RCLCPP_DEBUG(node_->get_logger(), "Z: +RotZ");
                    twist_msg->twist.angular.z = twist_angular_speed_;
                    break;
                case KEYCODE_X:  // Rotate CW around Z
                    if (!gello_paused_) { publish_twist = false; break; }
                    RCLCPP_DEBUG(node_->get_logger(), "X: -RotZ");
                    twist_msg->twist.angular.z = -twist_angular_speed_;
                    break;
                case KEYCODE_SPACE:  // Toggle Gello pause
                    toggleGelloPause();
                    publish_twist = false;
                    // Small delay to let the state update
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                    printStatus();
                    puts("");  // newline after status
                    break;
                case KEYCODE_ESC:
                    printf("\n");
                    RCLCPP_INFO(node_->get_logger(), "ESC pressed, quitting...");
                    // Ensure Gello is re-enabled before quitting
                    if (gello_paused_) {
                        RCLCPP_INFO(node_->get_logger(), "Re-enabling Gello before exit...");
                        toggleGelloPause();
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));
                    }
                    return;
                default:
                    publish_twist = false;
                    break;
            }

            if (publish_twist) {
                twist_pub_->publish(std::move(twist_msg));
            }
        }
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr switch_cmd_type_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gello_toggle_client_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr gello_paused_sub_;
    rclcpp::Node::SharedPtr node_;

    std::string twist_cmd_topic_;
    std::string twist_frame_;
    std::string gello_node_name_;
    double twist_linear_speed_;
    double twist_angular_speed_;
    int ros_queue_size_;
    bool gello_paused_;
};

void exit_sig_handler(int sig)
{
    (void)sig;
    keyboard_reader_.shutdown();
    rclcpp::shutdown();
    exit(-1);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared(
        "gello_keyboard_servo_node", node_options);

    RCLCPP_INFO(node->get_logger(), "namespace: %s", node->get_namespace());

    GelloKeyboardServoPub keyboard_servo_pub(node);
    signal(SIGINT, exit_sig_handler);
    keyboard_servo_pub.keyLoop();
    keyboard_reader_.shutdown();

    rclcpp::shutdown();

    RCLCPP_INFO(node->get_logger(), "gello_keyboard_servo_node terminated");

    return 0;
}
