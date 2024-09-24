from time import sleep
import time 
from evdev import InputDevice, categorize, ecodes
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3
import rclpy.node
from sensor_msgs.msg import Joy


class ControllerNode(Node):
    def __init__(self):
        super().__init__("controller_pub")      
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.subscriber_ = self.create_subscription(Joy, "/joy", self.subscribe, 10)
        self.get_logger().info("Hello from Controller node")
        self.timer = self.create_timer(0.5, self.publish)
        self.keys = ["Left", "Up", "L2", "R2"]#References ubuntu mate
        # self.keys = ["L2", "Left", "Up", "R2"]#For ubuntu
        self.controls = {}
        self.remap = lambda m, x, c : float(m*x + c)
        self.gear = 0
        self.acc_flag = 0
        self.dec_flag = 0
        self.hard_brake = 0 
        self.reverse_gear = 0
        self.vel = 0
        self.prev_time = time.time()

    #Publishing the message
    def publish(self):
        msg = Twist()
        acc = self.remapping()[0]
        dec = self.remapping()[1]
        self.vel = self.vel_compute(acc=acc, dec=dec, velocity=self.vel)
        self.get_logger().info(f"**************THIS IS THE SPEED: {self.vel}*******************")
        msg.linear.x = float(self.vel) #we shall print the value of velcity after conversion 
        msg.linear.y = float(0.0)
        msg.linear.z = float(0.0)
        msg.angular.z = self.remapping()[2]
        msg.angular.y = float(0.0)
        msg.angular.x = float(0.0)
        self.publisher_.publish(msg)


    def subscribe(self, joy):
        trimmed_list = joy.axes[2:6]
        self.controls = {self.keys[i]: trimmed_list[i] for i in range(len(trimmed_list))}

        #Ensuring that when we press X we are upshifting
        if joy.buttons[0] == 1 and self.acc_flag == 0:
            self.gear += 1
            self.get_logger().info(f"****UPSHIFTING****: {self.gear}")
            self.acc_flag = 1
            if self.gear >= 3:
                self.gear = 3
        elif joy.buttons[0] == 0 and self.acc_flag == 1:
            self.acc_flag = 0
        
        #Ensuring that when we press triangle we are downshifting
        if joy.buttons[3] == 1 and self.dec_flag == 0:
            self.gear -= 1
            self.get_logger().info(f"****DOWNSHIFTING****: {self.gear}")
            self.dec_flag = 1
            if self.gear <= 1:
                self.gear = 1
        elif joy.buttons[3] == 0 and self.dec_flag == 1:
            self.dec_flag = 0
        
        #Ensuring that when we O we are manual breaking
        if joy.buttons[1] == 1 and self.hard_brake == 0:
            self.gear = 1
            self.get_logger().info(f"****BREAKING****: {self.gear}")
            self.hard_brake = 1
        elif joy.buttons[1] == 0 and self.hard_brake == 1:
            self.hard_brake = 0
            
        #This is to shift to reverse
        if joy.buttons[2] == 1 and self.reverse_gear == 0 and self.gear == 1:
            self.gear = 0
            self.get_logger().info(f"****REVERSING****: {self.gear}")
            self.reverse_gear = 1
        elif joy.buttons[1] == 0 and self.reverse_gear == 1:
            self.reverse_gear = 0


        self.get_logger().info(f"{self.controls}")

    def remapping(self):
        acc = self.remap(-0.05, self.controls['L2'], 0.05)
        self.get_logger().info(f" ACC: {acc}")
        dec = self.remap(-0.075, self.controls['R2'], 0.075)
        self.get_logger().info(f"DEC: {dec}")
        dir = self.remap(2, self.controls['Left'], 0)
        self.get_logger().info(f"DIR: {dir}")
        return [acc, dec, dir]
    

    #Computing values from input to velocity
    def vel_compute(self, acc, dec, velocity) -> float:
        current_time = time.time()
        time_diff = current_time - self.prev_time
        if self.controls["L2"] != 1 and self.controls["R2"] == 1:#Making sure that its pure acceleration intended
            self.get_logger().info("########################--ACCELERATING--##################################")
            if self.gear == 1: 
                self.get_logger().info("########################--GEAR 1--##################################")
                velocity += (acc*time_diff)
                self.prev_time = current_time
                if velocity > 0.2: velocity = 0.2 #making sure the velocity doesn't pass a particular range
            elif self.gear == 2:
                self.get_logger().info("########################--GEAR 2--##################################")
                velocity += (acc*time_diff)
                self.prev_time = current_time
                if velocity > 0.4: velocity = 0.4 #making sure the velocity doesn't pass a particular range
            elif self.gear == 3:
                self.get_logger().info("########################--GEAR 3--##################################")
                velocity += (acc*time_diff)
                self.prev_time = current_time
                if velocity > 0.7: velocity = 0.7 #This is the maximum speed our robot can go

        elif self.controls["R2"] != 1 and self.controls["L2"] == 1: #Making sure that it is pure deceleration intended
            self.get_logger().info("#########################--DECELERATING--##################################")
            if self.gear == 1: 
                velocity -= (dec*time_diff)
                self.prev_time = current_time
                if velocity < 0.2: 
                    self.gear = 1
                elif 0.2 < velocity < 0.4:
                    self.gear = 2
                elif 0.4 < velocity < 0.7:
                    self.gear = 3
                elif velocity < 0.0:
                    velocity = 0.0
                    self.gear = 1
            elif self.gear == 0:
                self.get_logger().info("##########################--REVERSING--##################################")
                velocity -= (dec*time_diff)
                if velocity < -0.7: velocity = -0.7

        return velocity


def main(args = None):
    rclpy.init(args = args)
    node = ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()