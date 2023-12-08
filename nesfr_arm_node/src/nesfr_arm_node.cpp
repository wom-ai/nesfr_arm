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

#define LOG_INFO(fmt, ...)  \
    std::fprintf(stdout, "[info] " fmt "\n", ##__VA_ARGS__); std::fflush(stdout);

static void print_build_info(void)
{
    LOG_INFO("Built by %s on %s", __BUILD_USER__, __BUILD_HOSTNAME__);
    LOG_INFO("GCC __VERSION__=%s", __VERSION__);
    LOG_INFO("CPP STANDARD __cplusplus=%ld", __cplusplus);
    LOG_INFO("Build TIme: %s %s", __DATE__, __TIME__);
    LOG_INFO("Git Branch=[%s](%s)", __GIT_BRANCH__, __GIT_CURRENT_TAG__);
    LOG_INFO("    +-> commit %s", __GIT_COMMIT_HASH__);
}

class NesfrArmNode : public rclcpp::Node
{
    public:
        NesfrArmNode()
            : Node("nesfr_arm_node")
        {
            publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            timer_ = this->create_wall_timer(
                    100ms, std::bind(&NesfrArmNode::timer_callback, this));
            subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
                    "joy", 10, std::bind(&NesfrArmNode::joy_callback, this, _1));

            // setup for shared memory
            key_t cmdvel_shm_key = ftok("/tmp/cmdvel_shm_file",65);
            if(cmdvel_shm_key < 0){
                RCLCPP_ERROR(this->get_logger(), "Shared memory(/tmp/cmdvel_shm_file) not found. Please make sure NESFR System is running...");
                exit(-1);
            }
            int cmdvel_shmid = shmget(cmdvel_shm_key,1024,0666|IPC_CREAT);
            _cmdvel_shm = static_cast<float *>(shmat(cmdvel_shmid,(void*)0,0));
            _cmdvel_ts = reinterpret_cast<uint64_t *>(_cmdvel_shm);

            key_t wheel_cmd_shm_key = ftok("/tmp/wheel_cmd_shm_file",65);
            if(wheel_cmd_shm_key < 0){
                RCLCPP_ERROR(this->get_logger(), "Shared memory(/tmp/wheel_cmd_shm_file) not found. Please make sure NESFR System is running...");
                exit(-1);
            }
            int wheel_cmd_shmid = shmget(wheel_cmd_shm_key,1024,0666|IPC_CREAT);
            _wheel_cmd_shm = static_cast<float *>(shmat(wheel_cmd_shmid,(void*)0,0));
            _wheel_cmd_ts = reinterpret_cast<uint64_t *>(_wheel_cmd_shm);

            key_t arm_stats_shm_key = ftok("/tmp/arm_shm_file",65);
            if(arm_stats_shm_key < 0){
                RCLCPP_ERROR(this->get_logger(), "Shared memory(/tmp/arm_shm_file) not found. Please make sure NESFR System is running...");
                exit(-1);
            }
            int arm_stats_shmid = shmget(arm_stats_shm_key,1024,0666|IPC_CREAT);
            _arm_stats_shm = static_cast<float *>(shmat(arm_stats_shmid,(void*)0,0));
            _arm_stats_ts = reinterpret_cast<uint64_t *>(_arm_stats_shm);

            _min_arm_angle = this->declare_parameter<float>("joint_limits.shoulder_lift.min_position", 5.0f*(M_PI/180.0f));
            _max_arm_angle = this->declare_parameter<float>("joint_limits.shoulder_lift.max_position", 60.0f*(M_PI/180.0f));

            this->get_parameter<float>("joint_limits.shoulder_lift.min_position", _min_arm_angle);
            this->get_parameter<float>("joint_limits.shoulder_lift.max_position", _max_arm_angle);
            RCLCPP_INFO(this->get_logger(), "min/max_angle=(%f, %f)",_min_arm_angle, _max_arm_angle);

            _target_arm_angle = _arm_stats_shm[2];

            _joint_state_prefix = this->declare_parameter<std::string>("joint_state_prefix", "");
            this->get_parameter<std::string>("joint_state_prefix", _joint_state_prefix);

            RCLCPP_INFO(this->get_logger(), "Initialization completed");
        }

    private:
        void timer_callback()
        {
            auto message = sensor_msgs::msg::JointState();

            message.header.stamp = this->get_clock()->now();

            // publish joint_states
//            RCLCPP_DEBUG(this->get_logger(), "_arm_stats_shm[2]=%f (degrees)", _arm_stats_shm[2]*180.0f/M_PI);

//            std::string prefix = this->get_namespace();
//            prefix.erase(0, 1);
//            prefix += "/";

            message.name.push_back(_joint_state_prefix + "shoulder_lift");
            float shoulder_lift = -_arm_stats_shm[2];
            message.position.push_back(shoulder_lift);

            message.name.push_back(_joint_state_prefix + "elbow_joint");
            float elbow_joint_angle = -2.0f*shoulder_lift;
            message.position.push_back(elbow_joint_angle);

            message.name.push_back(_joint_state_prefix + "wrist_joint");
            message.position.push_back(shoulder_lift);

            // Dummy
            message.name.push_back(_joint_state_prefix + "main_cam_base_joint");
            message.position.push_back(0.0);
            message.name.push_back(_joint_state_prefix + "main_cam_pan_joint");
            message.position.push_back(0.0);
            message.name.push_back(_joint_state_prefix + "main_cam_tilt_joint");
            message.position.push_back(0.0);

            //RCLCPP_INFO(this->get_logger(), "publish %s: joint_state=%f", message.header.frame_id.c_str(), _target_arm_angle);
            publisher_->publish(message);
        }

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            // TODO: fix angle coordinates for arm joint control
            _target_arm_angle -= 0.02*msg->axes[XBOX_JOYSTICK_AXES_Y1];

            _target_arm_angle = std::clamp(_target_arm_angle, _min_arm_angle, _max_arm_angle);

            // set values into motors

            uint64_t ts = this->now().nanoseconds();
            memcpy(&_cmdvel_shm[0], &ts, sizeof(ts));
            _cmdvel_shm[5] = _target_arm_angle;
            _wheel_cmd_shm[6] = _target_arm_angle;

            //RCLCPP_INFO(this->get_logger(), "subscribe %s: _target_arm_angle=%f", msg->header.frame_id.c_str(), _target_arm_angle);
            RCLCPP_DEBUG(this->get_logger(), "_current_arm_angle=%10.6f _target_arm_angle=%10.6f (degrees)", _arm_stats_shm[2]*180.0f/M_PI,  _target_arm_angle*180.0f/M_PI);
        }

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;

        float _target_arm_angle;

        uint64_t _last_read_timestamp;

        float       *_cmdvel_shm;
        uint64_t    *_cmdvel_ts;

        float       *_wheel_cmd_shm;
        uint64_t    *_wheel_cmd_ts;

        float       *_arm_stats_shm;
        uint64_t    *_arm_stats_ts;

        float _min_arm_angle;
        float _max_arm_angle;

        std::string _joint_state_prefix = "";

};

int main(int argc, char * argv[])
{
    print_build_info();
    rclcpp::init(argc, argv);
    rclcpp::Node::SharedPtr node_ptr = std::make_shared<NesfrArmNode>();
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
