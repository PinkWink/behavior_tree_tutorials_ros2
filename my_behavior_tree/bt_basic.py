import rclpy
import time
import py_trees_ros
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees.decorators import Inverter
from py_trees import logging as log_tree

class Action(Behaviour):
    def __init__(self, name):
        super(Action, self).__init__(name)
        self.remained_dist = 3.0

    def update(self):
        self.logger.debug(f"Action::update {self.name} remained_dist: {self.remained_dist}")
        time.sleep(1.0)
        self.remained_dist -= 1.0

        if self.remained_dist <= 0.0:
            self.remained_dist = 3.0
            return Status.SUCCESS
        else:
            return Status.RUNNING

class ConditionCheckBatteryLow(Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.remained_battery = 100.0

    def update(self):
        self.logger.debug(f"Condition::update {self.name} remained_battery: {self.remained_battery}")
        time.sleep(1.0)
        self.remained_battery -= 30.0
        
        if self.remained_battery <= 0.0:
            self.logger.debug(f"Condition::update {self.name} BATTERY CHARGED!!!")
            self.remained_battery = 100.0
            return Status.SUCCESS
        else:
            return Status.FAILURE
    
class ConditionGetTarget(Behaviour):
    def __init__(self, name):
        super().__init__(name)

    def update(self):
        self.logger.debug(f"Condition::update {self.name}")
        time.sleep(1.0)
        
        return Status.SUCCESS

def make_bt():

    root = Sequence(name="sequence", memory=True)

    check_battery = Inverter(
                        'check_battery_low_inverter', 
                        ConditionCheckBatteryLow("check_battery_low")
    )

    get_target = ConditionGetTarget("get_target")
    move_to_target = Action("move_to_target")

    root.add_children(
        [
            check_battery,
            get_target,
            move_to_target
        ]
    )

    return root

def main():
    rclpy.init(args=None)
    log_tree.level = log_tree.Level.DEBUG

    root = make_bt()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )

    tree.setup(timeout=15)
    tree.tick_tock(period_ms=100.0) 

    rclpy.spin(tree.node)
    tree.shutdown()
    rclpy.try_shutdown()