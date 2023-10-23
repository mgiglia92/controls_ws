import rclpy
from rclpy.node import Node
from .ControllerInterface import PIDController_Beard, PIDControllerParameters
from geometry_msgs.msg import Twist, Vector3
from controller_interfaces.msg import State

class SimpleControllerNode(Node, PIDController_Beard):
    
    def __init__(self, node_name: str, params: PIDControllerParameters):
        Node.__init__(self, node_name=node_name)
        PIDController_Beard.__init__(self, params=params)
        self._pub = self.create_publisher(Vector3, '/input', 10)
        self._sub = self.create_subscription(State, '/state', self.state_callback, 10)
        print("here")
    
    def state_callback(self, msg: State):
        actual = msg.pose.position.x
        u = self.calculate_controls(1, actual)
        pub_msg = Vector3()
        pub_msg.x = u
        self._pub.publish(pub_msg)

def main():
    if not rclpy.ok(): rclpy.init()
    s = SimpleControllerNode('simple_controller', PIDControllerParameters())
    try:
        rclpy.spin(s)
    except:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
