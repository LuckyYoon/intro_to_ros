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


class bluerov2_sensors(Node):
    def __init__(self):
        super().__init__("battery_subscriber")
        self.battery = self.create_subscription(
            BatteryState, "mavros/battery", self.batteryCallback, 10
        )
        self.battery = BatteryState()
        self.get_logger().info("starting battery subscriber node")

        self.battery_timer = self.create_timer(5.0, self.voltageCheck())

        super().__init__("imu_subscriber")
        self.imu = self.create_subscription(
            Imu, "mavros/imu/data", self.imuCallback, 10
        )
        self.imu
        self.get_logger().info("starting imu subscriber node")

    def batteryCallback(self, msg):
        self.battery = msg
        self.get_logger().info(f"Battery Voltage\n\tx: ", self.battery.voltage)

    def imuCallback(self, msg):
        self.data = msg
        self.get_logger().info(f"IMU Time Stamp\n\tx: ", self.data.header.stamp.sec)

    def voltageCheck(self):
        if self.battery.voltage < 12:
            self.get_logger().info(f"Battery Voltage Low")
        else:
            self.get_logger().info(f"Battery Voltage Good")
