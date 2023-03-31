import sys
#import sysv_ipc
import time
import struct
import can
import math
# https://superfastpython.com/thread-mutex-lock/
from threading import Lock

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState

from geometry_msgs.msg import TransformStamped

from tf2_ros import TransformBroadcaster

import numpy as np

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q
#
# X: Horizontal, Y: Vertical
#
XBOX_JOYSTICK_AXES_MAX              = 8
XBOX_JOYSTICK_AXES_X0               = 0
XBOX_JOYSTICK_AXES_Y0               = 1
XBOX_JOYSTICK_AXES_X1               = 2
XBOX_JOYSTICK_AXES_Y1               = 3
XBOX_JOYSTICK_AXES_RIGHT_INDEX_GRIP = 4
XBOX_JOYSTICK_AXES_LEFT_INDEX_GRIP  = 5
XBOX_JOYSTICK_AXES_PAD_X            = 6
XBOX_JOYSTICK_AXES_PAD_Y            = 7

XBOX_JOYSTICK_BUTTONS_MAX           = 17
XBOX_JOYSTICK_BUTTONS_A             = 0
XBOX_JOYSTICK_BUTTONS_B             = 1
XBOX_JOYSTICK_BUTTONS_X             = 3
XBOX_JOYSTICK_BUTTONS_Y             = 4

#
# T-Motor specific constant
#
SET_POS_SPD = 6
CAN_EFF_FLAG = 0x80000000

class MotorAngleReader(can.Listener):
    def __init__(self, node):
        super().__init__()
        self.node = node

    def on_message_received(self, msg: can.Message):

#        self.node.get_logger().info(msg)

#        data_unpack = struct.unpack('B'*8, msg.data)
#        print(data_unpack)
#        print(data_unpack[0]*256+data_unpack[1])
        #print(data_unpack[1]*256+data_unpack[0])
        self.node.lock.acquire()
        data_unpack = struct.unpack('>HHHH', msg.data)
        self.node.current_arm_angle_ = data_unpack[0]*0.1*math.pi/180.0

        self.node.get_logger().debug('self.node.current_arm_angle_={} (degrees)'.format(self.node.current_arm_angle_*180.0/math.pi))

        self.node.lock.release()

class NesfrArmOnlyNode(Node):

    def __init__(self):
        super().__init__('nesfr_arm_only_node')
        self.prefix = self.get_namespace()[1:] + '/'

        self.lock = Lock()
        #
        # reference: https://m.blog.naver.com/techref/221999446630
        #
        self.bus = can.Bus(interface='socketcan',
                      channel='can0',
                      receive_own_messages=False)

        self._current_arm_angle = None # target angle to move
        self.current_arm_angle_ = None # current angle

        self.notifier = can.Notifier(self.bus, [MotorAngleReader(self)])
        # read angle from motor

        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.subscription = self.create_subscription(Joy, 'joy', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        #
        # reference: http://semanchuk.com/philip/sysv_ipc/
        #
#        self.cmdvel_shm_key = sysv_ipc.ftok("/tmp/cmdvel_shm_file",65)
#        if self.cmdvel_shm_key < 0:
#            self.get_logger().error("Shared memory not found. Please make sure NESFR System is running...")
#        self.cmdvel_shm_memory = sysv_ipc.SharedMemory(self.cmdvel_shm_key, flags=sysv_ipc.IPC_CREAT, size=1024)
#

        self.declare_parameter('joint_limits.shoulder_lift.min_position', 5.0*(math.pi/180.0))
        self.declare_parameter('joint_limits.shoulder_lift.max_position', 60.0*(math.pi/180.0))

        #
        # reference: https://www.theconstructsim.com/how-to-use-ros2-parameters/
        #
        self.min_arm_angle = self.get_parameter('joint_limits.shoulder_lift.min_position').value
        self.max_arm_angle = self.get_parameter('joint_limits.shoulder_lift.max_position').value

        # dummy location broadcaster
        self.get_logger().info('>>> dummy location setup')
        self.declare_parameter('default_pose.position.x', 0.0)
        self.declare_parameter('default_pose.position.y', 0.0)
        self.declare_parameter('default_pose.position.z', 0.0)

        self.declare_parameter('default_pose.orientation.x', 0.0)
        self.declare_parameter('default_pose.orientation.y', 0.0)
        self.declare_parameter('default_pose.orientation.z', 0.0)
        self.get_logger().info('<<< dummy location setup')

        self.tf_broadcaster = TransformBroadcaster(self)

    def timer_callback(self):

        ####################################################################
        # dummy location broadcaster
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = self.prefix + 'base_link'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = self.get_parameter('default_pose.position.x').value
        t.transform.translation.y = self.get_parameter('default_pose.position.y').value
        t.transform.translation.z = self.get_parameter('default_pose.position.z').value

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(  self.get_parameter('default_pose.orientation.x').value,
                                    self.get_parameter('default_pose.orientation.y').value,
                                    self.get_parameter('default_pose.orientation.z').value)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

        ####################################################################
        # joint state publish
#        data = self.cmdvel_shm_memory.read()
#        print("type(data)={}".format(type(data)))
#
#        data_unpack = struct.unpack('f'*6, data[:4*6])
#        self.get_logger().info('data: "{}"'.format(data_unpack))
        self.get_logger().debug('timer_callback()')
        if self.current_arm_angle_ == None:
            return

        if self._current_arm_angle == None:
            self._current_arm_angle = self.current_arm_angle_

        message = JointState()
        # https://answers.ros.org/question/354203/timestamp-message-ros2-python/
        message.header.stamp = self.get_clock().now().to_msg()


        message.name.append(self.prefix + "shoulder_lift")

        self.lock.acquire()
        shoulder_lift = (self.min_arm_angle - self.current_arm_angle_);
        self.lock.release()
        message.position.append(shoulder_lift);

        message.name.append(self.prefix + "elbow_joint");
        elbow_joint_angle = -2.0*shoulder_lift;
        message.position.append(elbow_joint_angle);

        message.name.append(self.prefix + "wrist_joint");
        message.position.append(shoulder_lift);

        # Dummy
        message.name.append(self.prefix + "main_cam_base_joint");
        message.position.append(0.0);
        message.name.append(self.prefix + "main_cam_pan_joint");
        message.position.append(0.0);
        message.name.append(self.prefix + "main_cam_tilt_joint");
        message.position.append(0.0);

        self.publisher.publish(message);

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "{}"'.format(msg))
        # TODO: fix angle coordinates for arm joint control
        self.get_logger().debug('listener_callback()')

        if self._current_arm_angle == None:
            return

        self._current_arm_angle -= 0.02*msg.axes[XBOX_JOYSTICK_AXES_Y1];

        self._current_arm_angle = min(max(self._current_arm_angle, self.min_arm_angle), self.max_arm_angle);

        self.get_logger().debug('self._current_arm_angle={} (degrees)'.format(self._current_arm_angle*180.0/math.pi))

        # write angle to motors
        target_angle = self._current_arm_angle*180.0/math.pi*10000.0

        _can_id = 0x0
        max_rot_speed = 800
        max_rot_accel = 400
        can_id = _can_id |  (SET_POS_SPD << 8)
        data = struct.pack(">LHH", int(target_angle), max_rot_speed, max_rot_accel)
        message = can.Message(arbitration_id=can_id, is_extended_id=True, data=data)
        self.get_logger().debug(f'message={message}')
        try:
            self.bus.send(message, timeout=0.1)
        except Exception as e:
            self.get_logger().error('{}'.format(e))

    def stop(self):
        self.notifier.stop()
        self.bus.shutdown()

def main(args=None):
    rclpy.init(args=sys.argv)

    nesfr_arm_only_node = NesfrArmOnlyNode()
    nesfr_arm_only_node.get_logger().info('args={}'.format(sys.argv))

    try:
        rclpy.spin(nesfr_arm_only_node)
    except KeyboardInterrupt:
        nesfr_arm_only_node.get_logger().info(' shutting down by KeyboardInterrupt')
    except:
        nesfr_arm_only_node.get_logger().info(' shutting down by Exception')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    nesfr_arm_only_node.stop()
    nesfr_arm_only_node.destroy_node()
    rclpy.shutdown()
