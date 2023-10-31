import pybullet as p
import time
import pybullet_data
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from controller_interfaces.msg import State
import os
import traceback
from threading import Thread

class FrictionlessMass1D(Node):
   def __init__(self, node_name):
      super().__init__(node_name)
      self.input = Twist()
      self._pub = self.create_publisher(State, '/state', 10)
      self._sub = self.create_subscription(Twist, '/input', self.input_callback, 10)
      self._sub
      self.kill = False
      
   def input_callback(self, msg:Twist):
      self.input = msg
   
   def publish_state(self, pos0): 
      msg = State()
      msg.pose.position.x=pos0[0]
      self._pub.publish(msg)

class FrictionlessMass2D(Node):
   def __init__(self, node_name):
      super().__init__(node_name)
      self.input = Twist()
      self._pub = self.create_publisher(State, '/state', 10)
      self._sub = self.create_subscription(Twist, '/input', self.input_callback, 10)
      self._sub
      self.kill = False
      
   def input_callback(self, msg:Twist):
      self.input = msg
   
   def publish_state(self, pos0): 
      msg = State()
      msg.pose.position.x=pos0[0]
      msg.pose.position.y=pos0[1]
      self._pub.publish(msg)

def spin(node):
    try:
        while not node.kill:
            rclpy.spin(node)
    except:
        rclpy.shutdown()
        traceback.print_exc()


def main():
    physicsClient = p.connect(p.GUI)# p.DIRECT for non-graphical version

    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")
    cubeStartPos = [0,0,0.5]
    cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
    boxId = p.loadURDF(os.path.join(os.path.dirname(__file__), "myblock.urdf"),cubeStartPos, cubeStartOrientation)
    if not rclpy.ok(): rclpy.init()
    node = FrictionlessMass2D('mass')
    pos0=[0,0,0]
    pos1=pos0
    dt=0.005
    p.setTimeStep(dt)
    spin_thread = Thread(target=spin, args=[node])
    spin_thread.start()
    start = time.time_ns()
    cur = time.time_ns() - start
    prev = cur
    try:
        while True:
            cur = time.time_ns() - start
            if(((cur - prev)/float(1e9)) >= dt):
                # p.stepSimulation()
                pos1, rot = p.getBasePositionAndOrientation(boxId)
                p.applyExternalForce(boxId, -1, [node.input.linear.x, node.input.linear.y, node.input.linear.z], [0,0,0], p.LINK_FRAME)
                p.applyExternalTorque(boxId, -1, [node.input.angular.x, node.input.angular.y, node.input.angular.z], p.LINK_FRAME)
                # pos0 = pos1
                node.publish_state(pos1)
                node.get_logger().info(f"dt: {(cur - prev)}")
                prev = cur
                p.stepSimulation()
    except KeyboardInterrupt:
        traceback.print_exc()

    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    print(cubePos,cubeOrn)

    p.disconnect()
    node.kill = True

if __name__ == "__main__":
   main()
   exit()