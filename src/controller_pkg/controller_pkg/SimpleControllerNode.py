import rclpy
from rclpy.node import Node
from .ControllerInterface import PIDController_Beard, PIDControllerParameters, PIDController_Beard_Vectorized
from geometry_msgs.msg import Twist
from controller_interfaces.msg import State
import traceback
import numpy as np

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

class SimpleControllerNode2d(Node, PIDController_Beard_Vectorized):

    def __init__(self, node_name: str, params: list[PIDControllerParameters]):
        Node.__init__(self, node_name=node_name)
        PIDController_Beard_Vectorized.__init__(self, params=params)
        
        self.desired = np.array([1.0, 1.0])
        self.actual  = np.array([0.0, 0.0])
        
        self._pub = self.create_publisher(Twist, '/input', 10)
        self._sub_actual = self.create_subscription(State, '/state', self.state_callback, 10)
        self._sub_desired = self.create_subscription(State, '/setpoint', self.setpoint_callback, 10)
        self._timer = self.create_timer(params[0].Ts, callback=self.send_controller_output)
        

    def state_callback(self, msg: State):
        self.actual[0] = msg.pose.position.x
        self.actual[1] = msg.pose.position.y
    
    def setpoint_callback(self, msg: State):
        self.desired[0] = msg.pose.position.x
        self.desired[1] = msg.pose.position.y

    def send_controller_output(self):
        u = self.calculate_controls(self.desired, self.actual)
        pub_msg = Twist()
        pub_msg.linear.x = u[0]
        pub_msg.linear.y = u[1]
        self._pub.publish(pub_msg)

def main():
    if not rclpy.ok(): rclpy.init()
    s = SimpleControllerNode2d('simple_controller', [PIDControllerParameters(), PIDControllerParameters(kp=2)])
    try:
        rclpy.spin(s)
    except:
        traceback.print_exc()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
