#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <algorithm>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <net/if.h>

#include <sys/ioctl.h>

#include "string.h"
#include <unistd.h> // read() / write()

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

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

#define SET_POS_SPD                 6
#define ARM_MOTOR_ID_MASK           0xFF
#define ARM_MOTOR_ID                0x00
#define ARM_MOTOR_MAX_ROT_SPEED     800
#define ARM_MOTOR_MAX_ROT_ACCEL     400

static inline float  deg_to_rad(float deg){
  return deg * M_PI / 180.0;
}

static inline float  rad_to_deg(float rad){
  return rad * 180.0 / M_PI;
}

#define PERF_MONITOR_INTERVAL 4000

std::atomic_bool system_on = {true};

class NesfrArmNode : public rclcpp::Node
{
    public:
        NesfrArmNode()
            : Node("nesfr_arm_node")
        {
            publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
            _timer = this->create_wall_timer(   100ms,
                                                std::bind(&NesfrArmNode::_timercallback,
                                                this));
            _subscription = this->create_subscription<sensor_msgs::msg::Joy>(   "joy",
                                                                                10,
                                                                                std::bind(&NesfrArmNode::joy_callback,
                                                                                this, _1));

            _min_arm_angle = this->declare_parameter<float>("joint_limits.shoulder_lift.min_position", 5.0f*(M_PI/180.0f));
            _max_arm_angle = this->declare_parameter<float>("joint_limits.shoulder_lift.max_position", 60.0f*(M_PI/180.0f));

            this->get_parameter<float>("joint_limits.shoulder_lift.min_position", _min_arm_angle);
            this->get_parameter<float>("joint_limits.shoulder_lift.max_position", _max_arm_angle);
            RCLCPP_INFO(this->get_logger(), "min/max_angle=(%f, %f)",_min_arm_angle, _max_arm_angle);

            _tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            
            this->declare_parameter<double>("default_pose.position.x", 0.0);
            this->declare_parameter<double>("default_pose.position.y", 0.0);
            this->declare_parameter<double>("default_pose.position.z", 0.0);

            this->declare_parameter<double>("default_pose.orientation.x", 0.0);
            this->declare_parameter<double>("default_pose.orientation.y", 0.0);
            this->declare_parameter<double>("default_pose.orientation.z", 0.0);

            tf2::Quaternion q;
            double x, y, z, ox, oy, oz;
            std::string prefix = this->get_namespace();
            prefix.erase(0, 1);
            prefix += "/";

            default_transform_stamped.header.frame_id = "map";
            default_transform_stamped.child_frame_id = prefix + "base_link";

            this->get_parameter_or<double>("default_pose.position.x", x, 0.0);
            this->get_parameter_or<double>("default_pose.position.y", y, 0.0);
            this->get_parameter_or<double>("default_pose.position.z", z, 0.0);

            this->get_parameter_or<double>("default_pose.orientation.ox", ox, 0.0);
            this->get_parameter_or<double>("default_pose.orientation.oy", oy, 0.0);
            this->get_parameter_or<double>("default_pose.orientation.oz", oz, 0.0);

            default_transform_stamped.transform.translation.x = x;
            default_transform_stamped.transform.translation.y = y;
            default_transform_stamped.transform.translation.z = z;

            q.setRPY(ox, oy, oz);

            default_transform_stamped.transform.rotation.x = q.x();
            default_transform_stamped.transform.rotation.y = q.y();
            default_transform_stamped.transform.rotation.z = q.z();
            default_transform_stamped.transform.rotation.w = q.w();
        }

    private:
        void _timercallback()
        {
            // publish default transform of the stationary arm robot
            default_transform_stamped.header.stamp = this->get_clock()->now();
            _tf_broadcaster->sendTransform(default_transform_stamped);

            // publish joint_states
            if (isnan(_current_arm_angle))
                return;
            auto message = sensor_msgs::msg::JointState();
            message.header.stamp = this->get_clock()->now();

            std::string prefix = this->get_namespace();
            prefix.erase(0, 1);
            prefix += "/";

            message.name.push_back(prefix + "shoulder_lift");
            float shoulder_lift = - _current_arm_angle.load();
            message.position.push_back(shoulder_lift);

            message.name.push_back(prefix + "elbow_joint");
            float elbow_joint_angle = -2.0f*shoulder_lift;
            message.position.push_back(elbow_joint_angle);

            message.name.push_back(prefix + "wrist_joint");
            message.position.push_back(shoulder_lift);

            // Dummy
            message.name.push_back(prefix + "main_cam_base_joint");
            message.position.push_back(0.0);
            message.name.push_back(prefix + "main_cam_pan_joint");
            message.position.push_back(0.0);
            message.name.push_back(prefix + "main_cam_tilt_joint");
            message.position.push_back(0.0);

            //RCLCPP_INFO(this->get_logger(), "publish %s: joint_state=%f", message.header.frame_id.c_str(), _target_arm_angle);
            publisher_->publish(message);
        }

        std::unique_ptr<tf2_ros::TransformBroadcaster> _tf_broadcaster;
        rclcpp::TimerBase::SharedPtr _timer;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;

        void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
        {
            if (isnan(_current_arm_angle))
                return;
            // TODO: fix angle coordinates for arm joint control
            float target_arm_angle = _target_arm_angle.load() - 0.02*msg->axes[XBOX_JOYSTICK_AXES_Y1];
            _target_arm_angle.store(std::clamp(target_arm_angle, _min_arm_angle, _max_arm_angle));

            //RCLCPP_DEBUG(this->get_logger(), "_target_arm_angle=%10.6f (degrees)", _target_arm_angle*180.0f/M_PI);
        }

        rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr _subscription;

        std::atomic<float> _target_arm_angle = {std::nanf("not initialized")}; // target angle to move
        std::atomic<float> _current_arm_angle = {std::nanf("not initialized")}; // current angle as result

        float _min_arm_angle;
        float _max_arm_angle;

        geometry_msgs::msg::TransformStamped default_transform_stamped;
       
        //-----------------------------------------------------------------------------
        // can ctrl part
        //-----------------------------------------------------------------------------

    private:

        unsigned int _can_id = ARM_MOTOR_ID;
        int _can_fd = -1;
        std::string _name = "can0";

        uint16_t _max_rot_speed = ARM_MOTOR_MAX_ROT_SPEED;   // max rotaional speed in erpm
        uint16_t _max_rot_accel = ARM_MOTOR_MAX_ROT_ACCEL;   // max rotational acceleration in erpm

        int _init_can(void) {
            struct sockaddr_can addr;
            struct ifreq ifr;

            _can_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
            if(_can_fd < 0) {
                perror("Cannot open CAN Socket!");
                return (-1);
            };

            strcpy(ifr.ifr_name, _name.c_str());
            ioctl(_can_fd, SIOCGIFINDEX, &ifr);

            memset(&addr, 0, sizeof(addr));
            addr.can_family = AF_CAN;
            addr.can_ifindex = ifr.ifr_ifindex;

            struct timeval timeout;
            timeout.tv_sec = 1;
            timeout.tv_usec = 0;

            int err = setsockopt(_can_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);
            if(err < 0){
                RCLCPP_ERROR(this->get_logger(), "Failed to setsockopt CAN socket! %s(%d)", strerror(errno), errno);
                return (-1);
            }

            //
            // references:
            //  - https://docs.huihoo.com/doxygen/linux/kernel/3.7/structcan__filter.html
            //
            struct can_filter rfilter;
            rfilter.can_id = ARM_MOTOR_ID;
            rfilter.can_mask = ARM_MOTOR_ID_MASK;
            err = setsockopt(_can_fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
            if(err < 0){
                RCLCPP_ERROR(this->get_logger(), "Failed to setsockopt CAN socket! %s(%d)", strerror(errno), errno);
                return (-1);
            }

            err = bind(_can_fd, (struct sockaddr *) &addr, sizeof(addr));
            if(err < 0){
                RCLCPP_ERROR(this->get_logger(), "Failed to bind CAN socket! %s(%d)", strerror(errno), errno);
                return (-1);
            }

            return 0;
        }

        int _read_arm_angle(float &angle) {
            struct can_frame response;
            unsigned int nbytes = read(_can_fd, &response, sizeof(struct can_frame));
            if(nbytes < sizeof(struct can_frame)){
                RCLCPP_ERROR(this->get_logger(), "Failed to read CAN socket! %d", errno);
                return -1;
            }

            if ((response.can_id&ARM_MOTOR_ID_MASK) == _can_id) {
                int16_t pos_int = response.data[0] << 8 | response.data[1];
                angle = deg_to_rad(0.1f*pos_int);
            } else {
                //RCLCPP_ERROR(this->get_logger(), "motor_id=%d", response.can_id&ARM_MOTOR_ID_MASK);
                return -1;
            }
            return 0;
        }

        int _write_arm_angle(const float angle) {
            struct can_frame cmd;
            memset(&cmd, 0, sizeof(cmd));
            cmd.can_id = _can_id | CAN_EFF_FLAG | (SET_POS_SPD << 8);

            float angle_ = rad_to_deg(angle) * 10000.0f;
            int32_t int_angle = static_cast<int32_t>(angle_);
            cmd.data[0] = int_angle >> 24;
            cmd.data[1] = int_angle >> 16;
            cmd.data[2] = int_angle >> 8;
            cmd.data[3] = int_angle;
            cmd.data[4] = _max_rot_speed >> 8;
            cmd.data[5] = 0xFF&_max_rot_speed;
            cmd.data[6] = _max_rot_accel >> 8;
            cmd.data[7] = 0xFF&_max_rot_accel;
            cmd.can_dlc = 8;
            unsigned int nbytes = write(_can_fd, &cmd, sizeof(struct can_frame));
            if(nbytes < sizeof(struct can_frame)){
                RCLCPP_ERROR(this->get_logger(), "Failed to write CAN socket! %d", errno);
                return -1;
            }

            return 0;
        }

    public:
        void can_ctrl_thread_func(void) {
            fprintf(stderr, ">>> CAN Ctrl Thread\n");
            if (_init_can() < 0) {
                system_on.store(false);
                return;
            }

            try {
                unsigned int tc_frame_count = 0;
                auto b = std::chrono::high_resolution_clock::now();
                while(system_on) {
                    float angle;
                    if(_read_arm_angle(angle) < 0) {
                        RCLCPP_ERROR(this->get_logger(), "_read_arm_angle() failed");
                        return;
                    }
                    _current_arm_angle.store(angle);

                    //RCLCPP_DEBUG(this->get_logger(), "_current_arm_angle=%10.6f (degrees)", _current_arm_angle*180.0f/M_PI);
                    RCLCPP_DEBUG(this->get_logger(), "_current_arm_angle=%10.6f _target_arm_angle=%10.6f (degrees)", _current_arm_angle*180.0f/M_PI, _target_arm_angle*180.0f/M_PI);

                    if (isnan(_target_arm_angle))
                        _target_arm_angle.store(std::clamp(_current_arm_angle.load(), _min_arm_angle, _max_arm_angle));
                    if (_write_arm_angle(_target_arm_angle.load()) < 0) {
                        RCLCPP_ERROR(this->get_logger(), "_write_arm_angle() failed");
                        return;
                    }

                    tc_frame_count++;
                    auto e = std::chrono::high_resolution_clock::now();
                    double elapsed = std::chrono::duration<double, std::milli>(e - b).count();
                    if (elapsed > PERF_MONITOR_INTERVAL) {
                        float fps = 1000.0*tc_frame_count/elapsed;
                        RCLCPP_INFO(this->get_logger(), "\t>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
                        RCLCPP_INFO(this->get_logger(), "\t    Read fps: %6f hz\telapsed= %9f milliseconds/ %d", fps, elapsed, tc_frame_count);
                        RCLCPP_INFO(this->get_logger(), "\t>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
                        //reset
                        tc_frame_count = 0;
                        b = std::chrono::high_resolution_clock::now();
                    }
                }
            } catch (std::exception &e) {
                RCLCPP_ERROR(this->get_logger(), "%s", e.what());
            }
                
            system_on.store(false);

            fprintf(stderr, "<<< CAN Ctrl Thread\n");
        }

        void stop_can_ctrl_thread_func(void) {
            system_on.store(false);
        }
};


int main(int argc, char * argv[])
{
    fprintf(stderr, ">>> Main Thread\n");
    rclcpp::init(argc, argv);

    std::shared_ptr<NesfrArmNode> node_ptr = std::make_shared<NesfrArmNode>();
    std::shared_ptr<std::thread> can_ctrl_thread_ptr = std::make_shared<std::thread>(&NesfrArmNode::can_ctrl_thread_func, node_ptr);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    rclcpp::WallRate rate(60);
    while(system_on && rclcpp::ok()) {
        rclcpp::spin_some(node_ptr);
    }

    node_ptr->stop_can_ctrl_thread_func();
    can_ctrl_thread_ptr->join();
    rclcpp::shutdown();

    fprintf(stderr, "<<< Main Thread\n");
    return 0;
}
