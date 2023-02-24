#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
using namespace std::chrono_literals;
using std::placeholders::_1;

//
// X: Horizontal, Y: Vertical
//
#define XBOX_JOYSTICK_AXES_MAX              8
#define XBOX_JOYSTICK_AXES_X0               0
#define XBOX_JOYSTICK_AXES_Y0               1
#define XBOX_JOYSTICK_AXES_X1               2
#define XBOX_JOYSTICK_AXES_Y1               3
#define XBOX_JOYSTICK_AXES_RIGHT_INDEX_GRIP 4
#define XBOX_JOYSTICK_AXES_LEFT_INDEX_GRIP  5
#define XBOX_JOYSTICK_AXES_PAD_X            6
#define XBOX_JOYSTICK_AXES_PAD_Y            7

#define XBOX_JOYSTICK_BUTTONS_MAX           17
#define XBOX_JOYSTICK_BUTTONS_A             0
#define XBOX_JOYSTICK_BUTTONS_B             1
#define XBOX_JOYSTICK_BUTTONS_X             3
#define XBOX_JOYSTICK_BUTTONS_Y             4

class NesfrArmNode : public rclcpp::Node
{
    public:
        NesfrArmNode()
            : Node("nesfr_arm_only_node")
        {
            publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            timer_ = this->create_wall_timer(
                    500ms, std::bind(&NesfrArmNode::timer_callback, this));
            subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
                    "joy", 10, std::bind(&NesfrArmNode::joy_callback, this, _1));

            // setup for shared memory
            key_t cmdvel_shm_key = ftok("/tmp/cmdvel_shm_file",65);
            if(cmdvel_shm_key < 0){
                RCLCPP_ERROR(this->get_logger(), "Shared memory not found. Please make sure NESFR System is running...");
                exit(-1);
            }
            int cmdvel_shmid = shmget(cmdvel_shm_key,1024,0666|IPC_CREAT);
            _cmdvel_shm = static_cast<float *>(shmat(cmdvel_shmid,(void*)0,0));
            _cmdvel_ts = reinterpret_cast<uint64_t *>(_cmdvel_shm);

            key_t wheel_cmd_shm_key = ftok("/tmp/cmdvel_shm_file",65);
            if(wheel_cmd_shm_key < 0){
                RCLCPP_ERROR(this->get_logger(), "Shared memory not found. Please make sure NESFR System is running...");
                exit(-1);
            }
            int wheel_cmd_shmid = shmget(wheel_cmd_shm_key,1024,0666|IPC_CREAT);
            _wheel_cmd_shm = static_cast<float *>(shmat(wheel_cmd_shmid,(void*)0,0));
            _wheel_cmd_ts = reinterpret_cast<uint64_t *>(_wheel_cmd_shm);

            this->declare_parameter("joint_limits.shoulder_lift.min_position");
            this->declare_parameter("joint_limits.shoulder_lift.max_position");

            this->get_parameter_or<float>("joint_limits.shoulder_lift.min_position", _min_arm_angle, 5.0f);
            this->get_parameter_or<float>("joint_limits.shoulder_lift.max_position", _max_arm_angle, 60.0f);
            RCLCPP_INFO(this->get_logger(), "min/max_angle=(%f, %f)",_min_arm_angle, _max_arm_angle);
        }

    private:
        void timer_callback()
        {
            auto message = sensor_msgs::msg::JointState();
            message.header.frame_id = "nesfr_arm_only";
            RCLCPP_INFO(this->get_logger(), "publish %s: joint_state=%f", message.header.frame_id.c_str(), _current_arm_angle);
            publisher_->publish(message);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
        {

            _current_arm_angle += 0.02f*msg->axes[XBOX_JOYSTICK_AXES_Y1];

            _current_arm_angle = std::clamp(_current_arm_angle, _min_arm_angle, _max_arm_angle);

            _cmdvel_shm[5] = _current_arm_angle;
            _wheel_cmd_shm[6] = _current_arm_angle;

            RCLCPP_INFO(this->get_logger(), "subscribe %s: _current_arm_angle=%f", msg->header.frame_id.c_str(), _current_arm_angle);
        }

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

        float _current_arm_angle;

        uint64_t _last_read_timestamp;
        float* _cmdvel_shm;
        uint64_t* _cmdvel_ts;

        float* _wheel_cmd_shm;
        uint64_t* _wheel_cmd_ts;

        float _min_arm_angle = 5.0f;
        float _max_arm_angle = 60.0f;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node_ptr = std::make_shared<NesfrArmNode>();
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
