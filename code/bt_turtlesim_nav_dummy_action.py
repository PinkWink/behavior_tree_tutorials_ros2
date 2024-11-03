import py_trees
import py_trees_ros
import rclpy

from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.decorators import Inverter
from py_trees import logging as log_tree
from my_behavior_tree_msgs.msg import PosError
from rclpy.node import Node
from geometry_msgs.msg import Twist
from my_behavior_tree.bt_turtlesim_nav_classes import CheckTargetInRange, CheckDistanceErrorNegative
from my_behavior_tree.bt_turtlesim_nav_classes import CheckAngleInRange, ActionDummy

def move_forward_seq_mode():
    move_forward = ActionDummy('action_move_forward') 

    move_forward_seq = py_trees.composites.Sequence(name="move_forward_seq", memory=True)
    move_forward_seq.add_child(move_forward)

    return move_forward_seq

def arrival_check_seq_mode():
    is_dist_in_range_2 = CheckTargetInRange('check_target_in_range')
    rotate_to_target_direction = ActionDummy('action_rotate_to_target_direction')

    arrival_check_seq = py_trees.composites.Sequence(name="arrival_check_seq", memory=True)
    arrival_check_seq.add_children([is_dist_in_range_2, rotate_to_target_direction])

    return arrival_check_seq

def misaligned_target_sel_mode():
    is_angle_in_range = CheckAngleInRange('check_angle_in_range')
    is_not_angle_in_range = Inverter(name="is_not_angle_in_range", child=is_angle_in_range)
    rotate_forward = ActionDummy('action_rotate_forward')

    misaligned_target_sel = py_trees.composites.Sequence(name="misaligned_target_sel", memory=True)
    misaligned_target_sel.add_children([is_not_angle_in_range, rotate_forward])

    return misaligned_target_sel

def aligned_target_sel_mode():
    arrival_check_seq = arrival_check_seq_mode()
    move_forward_seq = move_forward_seq_mode()

    aligned_target_sel = py_trees.composites.Selector(name="aligned_target_sel", memory=True)
    aligned_target_sel.add_children([arrival_check_seq, move_forward_seq])

    return aligned_target_sel

def forward_target_sel_mode():
    misaligned_target_sel = misaligned_target_sel_mode()
    aligned_target_sel = aligned_target_sel_mode()

    forward_target_sel = py_trees.composites.Selector(name="forward_target_sel", memory=True)
    forward_target_sel.add_children([misaligned_target_sel, aligned_target_sel])

    return forward_target_sel

def reverse_target_seq_mode():
    get_errors = py_trees_ros.subscribers.ToBlackboard(
        name="turtle_erros",
        topic_name="/errors_pos", 
        topic_type=PosError,
        blackboard_variables="turtle_erros",
        qos_profile=py_trees_ros.utilities.qos_profile_unlatched(),
    )
    is_dist_err_neg = CheckDistanceErrorNegative('check_distance_error_negative') 
    is_dist_in_range_1 = CheckTargetInRange('check_target_in_range')
    is_not_dist_in_range = Inverter(name="is_not_dist_in_range", child=is_dist_in_range_1)
    rotate_to_reverse = ActionDummy('action_rotate_reverse')

    reverse_target_seq = py_trees.composites.Sequence(name="reverse_target_seq", memory=True)
    reverse_target_seq.add_children([get_errors, is_dist_err_neg, is_not_dist_in_range, rotate_to_reverse])

    return reverse_target_seq

def turtlesim_nav_root():
    forward_target_sel = forward_target_sel_mode()
    reverse_target_seq = reverse_target_seq_mode()
    
    root = py_trees.composites.Selector(name="main_tree_root", memory=True)
    root.add_children([reverse_target_seq, forward_target_sel])
    
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
