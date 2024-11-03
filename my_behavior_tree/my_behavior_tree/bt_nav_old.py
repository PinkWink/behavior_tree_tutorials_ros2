import py_trees
import py_trees_ros
import rclpy
import time
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.decorators import Inverter
from py_trees import logging as log_tree
from my_behavior_tree_msgs.msg import PosError
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVel(Node):
    def __init__(self):
        super().__init__("cmd_vel_pub")
        self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    def pub_cmd_vel(self, linear_vel, angular_vel):
        msg = Twist()
        msg.linear.x = linear_vel
        msg.angular.z = angular_vel
        self.publisher_.publish(msg)

class ActionRotate(Behaviour):
    def __init__(self, name):
        super(ActionRotate, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="turtle_erros", access=py_trees.common.Access.READ)
        self.cmd_vel = CmdVel()

    def update(self):
        self.logger.debug(f"Action::update {self.name}")
        msg = self.blackboard.turtle_erros.angular_error

        if msg > 0:
            ang_ref = 0.1
        else:
            ang_ref = -0.1

        self.cmd_vel.pub_cmd_vel(0., ang_ref)
        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Action::terminate {self.name} to {new_status}")

class ActionMoveForward(Behaviour):
    def __init__(self, name):
        super(ActionMoveForward, self).__init__(name)

    def update(self):
        self.logger.debug(f"Action::update {self.name}")
        time.sleep(1)
        return Status.SUCCESS

    def terminate(self, new_status):
        self.logger.debug(f"Action::terminate {self.name} to {new_status}")

class CheckAngleInRange(Behaviour):
    def __init__(self, name="CheckAngleInRange"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="turtle_erros", access=py_trees.common.Access.READ)

    def update(self):
        msg = self.blackboard.turtle_erros.angular_error
        self.logger.debug(f"==> Blackboard angular_error: {msg}")

        if msg < 0.02:
            return Status.SUCCESS
        else:
            return Status.FAILURE

class CheckTargetInRange(Behaviour):
    def __init__(self, name="CheckTargetInRange"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(key="turtle_erros", access=py_trees.common.Access.READ)

    def update(self):
        msg = self.blackboard.turtle_erros.linear_error
        self.logger.debug(f"==> Blackboard linear_error: {msg}")



        if msg < 0.02:
            return Status.SUCCESS
        else:
            return Status.FAILURE


def turtlesim_nav_root() -> py_trees.behaviour.Behaviour:
    root = py_trees.composites.Sequence(name="main_tree_root", memory=True)
    root_nav = py_trees.composites.Selector(name="main_tree_root_nav", memory=True)

    get_errors = py_trees_ros.subscribers.ToBlackboard(
        name="turtle_erros",
        topic_name="/errors_pos", 
        topic_type=PosError,
        blackboard_variables="turtle_erros",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
    )

    root.add_children([get_errors, root_nav])

    move_forward_sequence = py_trees.composites.Sequence(name="move_forward_sequence", memory=True)
    rotate_sequence = py_trees.composites.Sequence(name="rotate_sequence", memory=True)
    rotate_target_sequence = py_trees.composites.Sequence(name="rotate_target_sequence", memory=True)

    #
    is_angle_in_range_for_forward = CheckAngleInRange()
    is_target_in_range_for_forward = CheckTargetInRange()
    move_forward = ActionMoveForward("move_forward")
    is_not_target_in_range_move_forward = Inverter(name="is_not_target_in_range_move_forward", 
                                                    child=is_target_in_range_for_forward)
    move_forward_sequence.add_children([
                            is_angle_in_range_for_forward, 
                            is_not_target_in_range_move_forward,
                            move_forward
    ])

    #
    is_angle_in_range_for_rotate = CheckAngleInRange()
    is_target_in_range_for_rotate = CheckTargetInRange()
    is_not_angle_in_range_rotate = Inverter(name="is_not_angle_in_range_rotate", 
                                                    child=is_angle_in_range_for_rotate)
    is_not_target_in_range_rotate = Inverter(name="is_not_target_in_range_rotate", 
                                                    child=is_target_in_range_for_rotate)
    rotate = ActionRotate("rotate")
    rotate_sequence.add_children([
                            is_not_angle_in_range_rotate, 
                            is_not_target_in_range_rotate,
                            rotate
    ])

    #
    is_angle_in_range_for_rotate_target = CheckAngleInRange()
    is_target_in_range_for_rotate_target = CheckTargetInRange()
    rotate_target = ActionRotate("rotate_target")
    rotate_target_sequence.add_children([
                            is_angle_in_range_for_rotate_target, 
                            is_target_in_range_for_rotate_target,
                            rotate_target
    ])

    #
    root_nav.add_children([move_forward_sequence, rotate_sequence, rotate_target_sequence])
    
    return root

def tutorial_main():
    rclpy.init(args=None)

    root = turtlesim_nav_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )

    tree.setup(timeout=15)
    log_tree.level = log_tree.Level.DEBUG
    tree.tick_tock(period_ms=500.0)

    rclpy.spin(tree.node)
    tree.shutdown()
    rclpy.try_shutdown()
