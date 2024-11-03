from py_trees.behaviour import Behaviour
from py_trees.common import Status
from rclpy.node import Node
from geometry_msgs.msg import Twist
import py_trees
import rclpy
import math

class ActionDummy(Behaviour):
    def __init__(self, name):
        super(ActionDummy, self).__init__(name)

    def update(self):
        self.logger.debug(f"Action::update {self.name}")
        return Status.SUCCESS 

class CmdVel(Node):
    def __init__(self):
        super().__init__("cmd_vel_pub")
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    def pub_cmd_vel(self, linear_vel, angular_vel):
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.publisher_.publish(msg)

class ActionRotateReverse(Behaviour):
    def __init__(self, name):
        super(ActionRotateReverse, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="turtle_erros", access=py_trees.common.Access.READ)
        self.target_angle_range = 0.2
        self.set_cmv_vel = CmdVel() 

    def update(self):
        self.logger.debug(f"Action::update {self.name}")

        ref_cmd_vel = -0.3
        self.set_cmv_vel.pub_cmd_vel(0.0, ref_cmd_vel)

        msgs = f'ref_cmd_vel: {ref_cmd_vel:.2f}'
        print("==> ActionRotateReverse", msgs)

        return Status.SUCCESS 
        

class ActionRotateForward(Behaviour):
    def __init__(self, name):
        super(ActionRotateForward, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="turtle_erros", access=py_trees.common.Access.READ)
        self.set_cmv_vel = CmdVel() 

    def update(self):
        self.logger.debug(f"Action::update {self.name}")

        msg = self.blackboard.turtle_erros.angular_error
        ref_cmd_vel = 0.3 if msg > 0 else -0.3
        self.set_cmv_vel.pub_cmd_vel(0.0, ref_cmd_vel)

        msgs = f'ref_cmd_vel: {ref_cmd_vel:.2f}'
        print("==> ActionRotateForward", msgs)

        return Status.SUCCESS

class ActionRotateToTargetDirection(Behaviour):
    def __init__(self, name):
        super(ActionRotateToTargetDirection, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="turtle_erros", access=py_trees.common.Access.READ)
        self.set_cmv_vel = CmdVel() 

    def update(self):
        self.logger.debug(f"Action::update {self.name}")

        msg = self.blackboard.turtle_erros.angular_error_to_target

        ref_cmd_vel = 0.3 if msg > 0 else -0.3
        self.set_cmv_vel.pub_cmd_vel(0.0, ref_cmd_vel)

        msgs = f'ref_cmd_vel: {ref_cmd_vel:.2f}'
        print("==> ActionRotateToTargetDirection", msgs)

        return Status.SUCCESS

class ActionMoveForward(Behaviour):
    def __init__(self, name):
        super(ActionMoveForward, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="turtle_erros", access=py_trees.common.Access.READ)
        self.set_cmv_vel = CmdVel() 

    def update(self):
        self.logger.debug(f"Action::update {self.name}")

        msg = self.blackboard.turtle_erros.linear_error
        if msg > 0:
            ref_cmd_vel = 0.3

        self.set_cmv_vel.pub_cmd_vel(ref_cmd_vel, 0.0)

        return Status.SUCCESS

class CheckDistanceErrorNegative(Behaviour):
    def __init__(self, name="DistanceErrorNegative"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="turtle_erros", access=py_trees.common.Access.READ)

    def update(self):
        self.logger.debug(f"Condition::update {self.name}")
        msg = self.blackboard.turtle_erros.linear_error

        if msg < 0: 
            return Status.SUCCESS
        else:
            return Status.FAILURE

class CheckAngleInRange(Behaviour):
    def __init__(self, name="CheckAngleInRange"):
        super().__init__(name)
        self.target_angle_range = 0.3
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="turtle_erros", access=py_trees.common.Access.READ)

    def update(self):
        self.logger.debug(f"Condition::update {self.name}")
        msg = abs(self.blackboard.turtle_erros.angular_error)

        if msg < self.target_angle_range:
            return Status.SUCCESS
        else:
            return Status.FAILURE

class CheckTargetInRange(Behaviour):
    def __init__(self, name="CheckTargetInRange"):
        super().__init__(name)
        self.target_dist_range = 0.2
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="turtle_erros", access=py_trees.common.Access.READ)

    def update(self):
        self.logger.debug(f"Condition::update {self.name}")
        msg = abs(self.blackboard.turtle_erros.linear_error)

        if msg < self.target_dist_range:
            return Status.SUCCESS
        else:
            return Status.FAILURE
        
class PID:
    def __init__(self):
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0
        self.max_state = 0.0
        self.min_state = 0.0
        self.pre_state = 0.0
        self.dt = 0.0
        self.integrated_state = 0.0
        self.pre_time = rclpy.Time.now()
        
    def process(self, state):
        self.dt = rclpy.Time.now() - self.pre_time

        if self.dt == 0.:
            state_D = 0.
        else:
            state_D = (state - self.pre_state) / self.dt

        state_I = state + self.integrated_state

        out = self.P*state + self.D*state_D + self.I*state_I * self.dt

        if out > self.max_state:
            out = self.max_state
        elif out < self.min_state:
            out = self.min_state

        self.pre_state = state
        self.integrated_state = state_I
        self.pre_time = rclpy.Time.now()

        return out