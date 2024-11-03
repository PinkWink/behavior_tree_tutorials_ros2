import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from my_behavior_tree_msgs.msg import PosError
import math

class CalcError(Node):
    def __init__(self):
        super().__init__('calc_error')
        self.subscription = self.create_subscription(
            Pose, '/turtle1/pose',
            self.turlte1_pose_callback, 10)

        self.goal_subscription = self.create_subscription(
            Pose, '/goal_pose_turtle',
            self.goal_pose_turtle_callback, 10)
        
        self.publisher = self.create_publisher(PosError, 'errors_pos', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.turtle_pose = Pose()
        self.errors = PosError() 

        self.goal_pose_turtle = Pose()
        self.goal_pose_turtle.x = 5.545
        self.goal_pose_turtle.y = 5.54445
        self.goal_pose_turtle.theta = 0.0

    def turlte1_pose_callback(self, msg):
        self.turtle_pose = msg

    def goal_pose_turtle_callback(self, msg):
        self.goal_pose_turtle = msg

    def calc_error(self, turtle_pose, goal_pose_turtle):
        x_distance = turtle_pose.x - goal_pose_turtle.x
        y_distance = turtle_pose.y - goal_pose_turtle.y

        angle_between = math.atan2(y_distance, x_distance) - math.pi

        angular_error = angle_between - turtle_pose.theta 

        while angular_error > 2*math.pi:
            angular_error -= 2*math.pi
        while angular_error < -2*math.pi:
            angular_error += 2*math.pi

        if angular_error < -math.pi:
            angular_error += 2*math.pi
            
        linear_error_distance = math.sqrt(x_distance**2 + y_distance**2)
        sign_linear_error = -math.cos(math.atan2(y_distance, x_distance) - turtle_pose.theta)

        linear_error = linear_error_distance * sign_linear_error

        print(f'angle_between: {angle_between*180./math.pi:.2f}, angular_error: {angular_error*180./math.pi:.2f}')
        
        return linear_error, angular_error
    
    def calc_angular_error_to_target(self, turtle_pose, goal_pose_turtle):
        error = goal_pose_turtle.theta - turtle_pose.theta

        return error

    def timer_callback(self):
        linear_error, angular_error = self.calc_error(self.turtle_pose, self.goal_pose_turtle)
        self.errors.linear_error = linear_error
        self.errors.angular_error = angular_error
        
        self.errors.angular_error_to_target = self.calc_angular_error_to_target(
                                                    self.turtle_pose, 
                                                    self.goal_pose_turtle)

        msgs = f'Goal pose: {self.goal_pose_turtle.x:.2f}, {self.goal_pose_turtle.y:.2f}'
        msgs += f' || Linear error: {self.errors.linear_error:.2f}' 
        msgs += f', || Angular error: {self.errors.angular_error*180./math.pi:.2f}'
        msgs += f', || Angular error to target: {self.errors.angular_error_to_target*180./math.pi:.2f}'

        self.get_logger().info(msgs)
        self.publisher.publish(self.errors)

def main(args=None):
    rclpy.init(args=args)
    calc_error = CalcError()
    rclpy.spin(calc_error)
    calc_error.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()