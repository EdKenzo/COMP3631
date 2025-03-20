import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.exceptions import ROSInterruptException
import signal
import time
import math


class SquareWalker(Node):
    def __init__(self):
        super().__init__('squarewalker')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz. rate will ensure consistent time before running again taking how long code runs into account
        self.side_length = 1.0  
        self.linear_speed = 0.2  
        self.angular_speed = math.pi / 2  # 90 degrees in radians

        
        
    def walk_forward(self, duration=0.5):
        twist_msg = Twist()
        twist_msg.linear.x = self.linear_speed # Forward with 0.2 m/s

        for _ in range(int(self.side_length / self.linear_speed * 10)):  # Calculate steps for side length
            self.publisher.publish(twist_msg)
            self.rate.sleep()

    def turn_left(self, duration=1.0):
        twist_msg = Twist()
        twist_msg.angular.z = self.angular_speed # 90 degrees per second

        for _ in range(10):  # Expect it to take 10 seconds to turn 90 degrees
            self.publisher.publish(twist_msg)
            self.rate.sleep()


    def stop(self):
        twist_msg = Twist()
        twist_msg.linear.x = 0.0  # Send zero velocity to stop the robot
        twist_msg.angular.z = 0.0  # Stop angular motion
        self.publisher.publish(twist_msg)

    def move_in_square(self):
        for _ in range(4):  # Perform four sides of a square
            self.walk_forward()  # Walk forward 
            self.turn_left()     # Turn left (90 degrees) 
        self.stop()
            
def main():
    def signal_handler(sig, frame):  # stops program when user ctrl c
        square_walker.stop()
        rclpy.shutdown()

    rclpy.init(args=None)
    square_walker = SquareWalker()

    signal.signal(signal.SIGINT, signal_handler)
    thread = threading.Thread(target=rclpy.spin, args=(square_walker,), daemon=True)
    thread.start()
    
    try:
        square_walker.move_in_square()
    except ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
