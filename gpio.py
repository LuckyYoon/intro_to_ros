import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time


class LEDBlinkerNode(Node):
    def __init__(self):
        super().__init__("electromagnet")
        self.pin = 16
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.OUT)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        GPIO.output(self.pin, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(self.pin, GPIO.LOW)
        time.sleep(1)


def main(args=None):
    rclpy.init(args=args)
    led_blinker_node = LEDBlinkerNode()
    rclpy.spin(led_blinker_node)

    GPIO.cleanup()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
