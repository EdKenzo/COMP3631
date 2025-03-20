import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.exceptions import ROSInterruptException
import signal


class CircleWalker(Node):
    def __init__(self):
        super().__init__('circlewalker')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz. rate will ensure consistent time before running again taking how long code runs into account
        self.radius = 1# can change
        self.angular_speed = 0.2
        self.linear_speed = self.radius * self.angular_speed
        
    def make_circle(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = self.linear_speed # Forward with 0.2 m/s
        desired_velocity.angular.z = self.angular_speed
        self.publisher.publish(desired_velocity)
 

    def stop(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.0  # Send zero velocity to stop the robot
        desired_velocity.angular.z = 0.0
        self.publisher.publish(desired_velocity)

def main():
    def signal_handler(sig, frame): # stops program when user ctrl c
        circle_walker.stop()
        rclpy.shutdown()

    rclpy.init(args=None)
    circle_walker = CircleWalker()

    signal.signal(signal.SIGINT, signal_handler)
    
    # run first walker in separate thread so we can concurrently execute other commands
    thread = threading.Thread(target=rclpy.spin, args=(circle_walker,), daemon=True)
    thread.start()
    iterations = int(20 * 3.14159 / circle_walker.angular_speed)  # Calculate iterations based on angular speed
    try: 
        for _ in range(iterations):  # Stop for a brief moment
            circle_walker.make_circle()
            circle_walker.rate.sleep()
    except ROSInterruptException:
        pass
    finally:
        circle_walker.stop()


if __name__ == "__main__":
    main()


