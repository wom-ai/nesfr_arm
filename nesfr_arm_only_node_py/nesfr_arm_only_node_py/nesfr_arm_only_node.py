import sysv_ipc
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class NesfrArmOnlyNode(Node):

    def __init__(self):
        super().__init__('nesfr_arm_only_node')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        #
        # reference: http://semanchuk.com/philip/sysv_ipc/
        #
        self.cmdvel_shm_key = sysv_ipc.ftok("/tmp/cmdvel_shm_file",65)
        if self.cmdvel_shm_key < 0:
            self.get_logger().error("Shared memory not found. Please make sure NESFR System is running...")
        self.cmdvel_shm_memory = sysv_ipc.SharedMemory(self.cmdvel_shm_key, flags=sysv_ipc.IPC_CREAT, size=1024)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        data = self.cmdvel_shm_memory.read()
        print("type(data)={}".format(type(data)))

        data_unpack = struct.unpack('f'*6, data[:4*6])
        self.get_logger().info('data: "{}"'.format(data_unpack))

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "{}"'.format(msg))


def main(args=None):
    rclpy.init(args=args)

    nesfr_arm_only_node = NesfrArmOnlyNode()

    try:
        rclpy.spin(nesfr_arm_only_node)
    except KeyboardInterrupt:
        nesfr_arm_only_node.get_logger().info(' shutting down by KeyboardInterrupt')

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    nesfr_arm_only_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
