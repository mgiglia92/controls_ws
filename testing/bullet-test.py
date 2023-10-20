import pybullet as p
import time
import pybullet_data
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import os

class FrictionlessMass(Node):
   def __init__(self, node_name):
      super().__init__(node_name)
      self.input = Vector3
      self._pub = self.create_publisher(Vector3, '/state', 10)
      self._sub = self.create_subscription(Vector3, '/input', self.input_callback, 10)
      self._sub
      
   def input_callback(self, msg:Vector3):
      self.input = msg
   
   def publish_state(self, position): 
      msg = Vector3(x=pos[0],y=pos[1],z=pos[2])
      self._pub.publish(msg)

physicsClient = p.connect(p.GUI)# p.DIRECT for non-graphical version

p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.5]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF(os.path.join(os.path.dirname(__file__), "myblock.urdf"),cubeStartPos, cubeStartOrientation)
rclpy.init()
node = FrictionlessMass('mass')

for i in range (10000):
   p.stepSimulation()
   time.sleep(1./240.)
   pos, rot = p.getBasePositionAndOrientation(boxId)
   p.applyExternalForce(boxId, -1, [(1-pos[0])*1.5,0,0], [0,0,0], p.WORLD_FRAME)
   node.publish_state(pos)

cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(cubePos,cubeOrn)

p.disconnect()
rclpy.shutdown()