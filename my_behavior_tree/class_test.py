import rclpy
from rclpy.node import Node
from my_behavior_tree.bt_turtlesim_nav_classes import EmptyNode

def main():
    rclpy.init(args=None)
    node = Node('test')
    test = EmptyNode()
    test.node_update()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
