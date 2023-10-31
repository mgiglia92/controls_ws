import rclpy
from rclpy.node import Node
from .ControllerInterface import PIDController_Beard, PIDControllerParameters
from geometry_msgs.msg import Twist
from controller_interfaces.msg import State
import traceback

class SimpleControllerNode(Node, PIDController_Beard):

    def __init__(self, node_name: str, params: PIDControllerParameters):
        Node.__init__(self, node_name=node_name)
        PIDController_Beard.__init__(self, params=params)
        
        self.desired = float(1.0)
        self.actual  = float()
        
        self._pub = self.create_publisher(Twist, '/input', 10)
        self._sub_actual = self.create_subscription(State, '/state', self.state_callback, 10)
        self._sub_desired = self.create_subscription(State, '/setpoint', self.setpoint_callback, 10)
        self._timer = self.create_timer(params.Ts, callback=self.send_controller_output)
        

    def state_callback(self, msg: State):
        self.actual = msg.pose.position.x
    
    def setpoint_callback(self, msg: State):
        self.desired = msg.pose.position.x

    def send_controller_output(self):
        u = self.calculate_controls(self.desired, self.actual)
        pub_msg = Twist()
        pub_msg.linear.x = u
        self._pub.publish(pub_msg)

class SimpleControllerNode2d(Node, PIDController_Beard):

    def __init__(self, node_name: str, params: PIDControllerParameters):
        Node.__init__(self, node_name=node_name)
        PIDController_Beard.__init__(self, params=params)
        
        self.desired = float(1.0)
        self.actual  = float()
        
        self._pub = self.create_publisher(Twist, '/input', 10)
        self._sub_actual = self.create_subscription(State, '/state', self.state_callback, 10)
        self._sub_desired = self.create_subscription(State, '/setpoint', self.setpoint_callback, 10)
        self._timer = self.create_timer(params.Ts, callback=self.send_controller_output)
        

    def state_callback(self, msg: State):
        self.actual = msg.pose.position.x
    
    def setpoint_callback(self, msg: State):
        self.desired = msg.pose.position.x

    def send_controller_output(self):
        u = self.calculate_controls(self.desired, self.actual)
        pub_msg = Twist()
        pub_msg.linear.x = u
        self._pub.publish(pub_msg)

def main():
    if not rclpy.ok(): rclpy.init()
    s = SimpleControllerNode('simple_controller', PIDControllerParameters())
    try:
        rclpy.spin(s)
    except:
        traceback.print_exc()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
