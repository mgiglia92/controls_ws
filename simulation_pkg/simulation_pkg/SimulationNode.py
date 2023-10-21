import pybullet as p
import time
import pybullet_data
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import os
import traceback
from threading import Thread

class FrictionlessMass(Node):
   def __init__(self, node_name):
      super().__init__(node_name)
      self.input = Vector3
      self._pub = self.create_publisher(Vector3, '/state', 10)
      self._sub = self.create_subscription(Vector3, '/input', self.input_callback, 10)
      self._sub
      
   def input_callback(self, msg:Vector3):
      self.input = msg
   
   def publish_state(self, pos0): 
      msg = Vector3(x=pos0[0],y=pos0[1],z=pos0[2])
      self._pub.publish(msg)


def spin():
    try:
        while True:
            rclpy.spin()
    except:
        traceback.print_exc()


def main():
    spin_thread = Thread(target=spin)
    physicsClient = p.connect(p.GUI)# p.DIRECT for non-graphical version

    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-10)
    planeId = p.loadURDF("plane.urdf")
    cubeStartPos = [0,0,0.5]
    cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
    boxId = p.loadURDF(os.path.join(os.path.dirname(__file__), "myblock.urdf"),cubeStartPos, cubeStartOrientation)
    rclpy.init()
    node = FrictionlessMass('mass')
    pos0=[0,0,0]
    pos1=pos0
    dt=0.01
    p.setTimeStep(dt)
    spin_thread.start()

    for i in range (10000):
        p.stepSimulation()
        time.sleep(dt)
        pos1, rot = p.getBasePositionAndOrientation(boxId)
        p.applyExternalForce(boxId, -1, [(1-pos0[0])*1.5 - 1*(pos1[0]-pos0[0])/dt,0,0], [0,0,0], p.WORLD_FRAME)
        pos0 = pos1
        node.publish_state(pos0)
        node.get_logger().info(f"Published: {pos0}")


    cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
    print(cubePos,cubeOrn)

    p.disconnect()

if __name__ == "__main__":
   main()