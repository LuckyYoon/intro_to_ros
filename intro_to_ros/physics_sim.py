#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
)


import numpy as np


# Problem 9
def calculate_auv2_acceleration(T, alpha, theta, mass):
    xComp = 0
    yComp = 0

    for i in T:
        xComp = xComp + np.cos(alpha) * i
        yComp = yComp + np.sin(alpha) * i

    return np.sqrt(xComp**2 + yComp**2) / mass


def calculate_auv2_angular_acceleration(T, alpha, L, l, inertia):
    distance = np.sqrt(L**2 + l**2)
    torque = distance * -T[0] * np.sin(alpha)
    torque = torque + distance * T[1] * np.sin(alpha)
    torque = torque + distance * -T[2] * np.sin(alpha)
    torque = torque + distance * T[3] * np.sin(alpha)
    return torque / inertia


# Problem 10
def simulate_auv2_motion(T, alpha, L, l, mass, inertia, dt, t_final, x0, y0, theta0):
    t = np.array[t_final / dt + 1]
    X = np.array[t_final / dt + 1]
    Y = np.array[t_final / dt + 1]
    theta = np.array[t_final / dt + 1]
    v = np.array[t_final / dt + 1]
    omega = np.array[t_final / dt + 1]
    a = np.array[t_final / dt + 1]

    # distance = np.sqrt(L**2 + l**2)
    t[0] = 0
    a[0] = 0
    omega[0] = 0
    theta[0] = theta0
    v[0] = 0
    X[0] = x0
    Y[0] = y0

    for i in range(1, t_final / dt + 1):
        t[i] = i / (1 / dt)
        a[i] = calculate_auv2_acceleration(T, alpha, theta[i])
        omega[i] = (
            omega[i - 1]
            + calculate_auv2_angular_acceleration(T, alpha, L, l, inertia) * dt
        )
        theta[i] = theta[i - 1] + omega[i] * dt
        v[i] = v[i - 1] + a[i] * dt
        X[i] = X[i - 1] + np.cos(theta[i]) * v[i]
        Y[i] = Y[i - 1] + np.sin(theta[i]) * v[i]

        return [X, Y]


T = np.ndarray(shape=(4,), dtype=float, buffer=np.array([2.0, 1.0, 2.0, 1.0]))
alpha = np.pi / 4
L = 0.3
l = 0.2
mass = 1.0
dt = 0.1
t_final = 10.0


class physics_sim(Node):
    def __init__(self):
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        super().__init__("pose2d_subscriber")
        self.pose2dSubscription = self.create_subscription(
            Pose2D, "physics/pose2d", self.pose2dCallback, qos_profile
        )
        self.get_logger().info("starting pose2d subscriber node")
        self.pose2d = Pose2D()

    def pose2dCallback(self, msg):
        self.pose2d = msg
        simulate_auv2_motion(
            T,
            alpha,
            L,
            l,
            mass,
            inertia=100,
            dt=0.1,
            t_final=10,
            x0=self.pose2d.x,
            y0=self.pose2d.y,
            theta0=self.pose2d.theta,
        )
        self.get_logger().info(f"Battery Voltage\n\tx: {self.battery.voltage}")

    def main(args=None):
        rclpy.init(args=args)
        node = physics_sim()

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
