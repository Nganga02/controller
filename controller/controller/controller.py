import asyncio
from time import sleep
from evdev import InputDevice, categorize, ecodes
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import rclpy.node
from sensor_msgs.msg import Joy


# #asynchrounous reading of the input
# async def read_input(dev, node):
#     async for ev in dev.async_read_loop():
#         if ev.type == ecodes.EV_ABS and ev.code in codes:
#             print(ev) 
#             node.publish()


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_pub")      
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.subscriber_ = self.create_subscription(Joy, "/joy", self.subscribe, 10)
        self.get_logger().info("Hello from Controller node")
        self.timer = self.create_timer(0.5, self.publish)
        self.keys = ["Left", "Up", "L2", "R2"]
        self.controls = {}
        self.remap = lambda m, x, c : float(m*x + c)

    #Publishing the message
    def publish(self):
        msg = Twist()
        msg.linear.x = self.remapping()[0]
        msg.linear.y = float(0.0)
        msg.linear.z = float(0.0)
        msg.angular.z = self.remapping()[2]
        msg.angular.y = float(0.0)
        msg.angular.x = float(0.0)
        self.publisher_.publish(msg)

    def subscribe(self, joy):
        trimmed_list = joy.axes[2:]
        self.controls = {self.keys[i]: trimmed_list[i] for i in range(len(trimmed_list))}
        # self.publish()
        self.get_logger().info(f"{self.controls}")

    def remapping(self):
        acc = self.remap(-1.25, self.controls['L2'], 1.25)
        self.get_logger().info(f"{acc}")
        dec = self.remap(2.5, self.controls['R2'], -2.5)
        self.get_logger().info(f"{dec}")
        dir = self.remap(2, self.controls['Left'], 0)
        self.get_logger().info(f"{dir}")
        return [acc, dec, dir]
    


def main(args = None):
    rclpy.init(args = args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()