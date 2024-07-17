#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)


import numpy as np

from mavros_msgs.msg import Mavlink

# pack and unpack functions to deal with the bytearray
from struct import pack, unpack


# Topic callback
def callback(data):
    # Check if message id is valid (I'm using SCALED_PRESSURE
    # and not SCALED_PRESSURE2)
    if data.msgid == 29:
        self.get_logger().info(rclpy.get_caller_id() + " Package: %s", data)
        # Transform the payload in a python string
        p = pack("QQ", *data.payload64)
        # Transform the string in valid values
        # https://docs.python.org/2/library/struct.html
        time_boot_ms, press_abs, press_diff, temperature = unpack("Iffhxx", p)


def listener():
    rospy.init_node("listener", anonymous=True)

    rospy.Subscriber("/mavlink/from", Mavlink, callback)

    rospy.spin()


if __name__ == "__main__":
    listener()


class bluerov2_sensors(Node):
    def __init__(self):
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        super().__init__("sensor_subscriber")
        self.batterySubscription = self.create_subscription(
            BatteryState, "mavros/battery", self.batteryCallback, qos_profile
        )
        self.get_logger().info("starting battery subscriber node")
        self.battery = BatteryState()
        self.battery.voltage = 12.6

        self.imuSubscription = self.create_subscription(
            Imu, "mavros/imu/data", self.imuCallback, qos_profile
        )
        self.imu = Imu()
        self.get_logger().info("starting imu subscriber node")

        self.battery_timer = self.create_timer(5.0, self.voltageCheck)

    def batteryCallback(self, msg):
        self.battery = msg
        self.get_logger().info(f"Battery Voltage\n\tx: {self.battery.voltage}")

    def imuCallback(self, msg):
        self.imu = msg
        self.get_logger().info(f"IMU Time Stamp\n\tx: {self.imu.header.stamp.sec}")

    def voltageCheck(self):
        print(self.battery.voltage)
        if self.battery.voltage < 12:
            self.get_logger().info("Battery Voltage Critical")
        elif self.battery.voltage < 12.6:
            self.get_logger().info("Battery Voltage Low")
        else:
            self.get_logger().info("Battery Voltage Good")


def main(args=None):
    rclpy.init(args=args)
    node = bluerov2_sensors()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
