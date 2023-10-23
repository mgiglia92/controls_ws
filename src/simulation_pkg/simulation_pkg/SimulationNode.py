import pybullet as p
import time
import pybullet_data
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from controller_interfaces.msg import State
import os
import traceback
from threading import Thread

class FrictionlessMass(Node):
   def __init__(self, node_name):
      super().__init__(node_name)
      self.input = Vector3()
      self._pub = self.create_publisher(State, '/state', 10)
      self._sub = self.create_subscription(Vector3, '/input', self.input_callback, 10)
      self._sub
      
   def input_callback(self, msg:Vector3):
      self.input = msg
   
   def publish_state(self, pos0): 
      msg = State()
      msg.pose.position.x=pos0[0]
      self._pub.publish(msg)


def spin(node):
    try:
        while True:
            rclpy.spin(node)
    except:
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
    node = FrictionlessMass('mass')
    pos0=[0,0,0]
    pos1=pos0
    dt=0.01
    p.setTimeStep(dt)
    spin_thread = Thread(target=spin, args=[node])
    spin_thread.start()

    for i in range (10000):
        p.stepSimulation()
        time.sleep(dt)
        pos1, rot = p.getBasePositionAndOrientation(boxId)
        p.applyExternalForce(boxId, -1, [node.input.x, 0, 0], [0,0,0], p.WORLD_FRAME)
        pos0 = pos1
        node.publish_state(pos0)
        node.get_logger().info(f"Published: {pos0}")


    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    print(cubePos,cubeOrn)

    p.disconnect()

if __name__ == "__main__":
   main()
   exit()