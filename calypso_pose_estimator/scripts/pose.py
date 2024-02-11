import rospy
from sensor_msgs.msg import Imu
from scipy.integrate import trapezoid
import time
import numpy as np
from tf.transformations import euler_from_quaternion
from dvl_msgs.msg import DVL
from calypso_pose_estimator.msg import pose

class cord:

  def __init__ (self):
    
      self.time=[]
      self.final=0
      self.acc = []
      self.vel = []
      self.start_time=time.time()

  def integrate(self, y, x):
      try:
          return trapezoid(y, x)
      except Exception as e:
          print(len(y),len(x))
          print("Sync failure !!")
  
  def get_imu_pose(self):
      
    vel = round(self.integrate(self.acc,self.time), 2)
    self.vel.append(vel)
    return self.integrate(self.vel, self.time)
  
  def get_dvl_pose(self):
    self.time.append(time.time()-self.start_time)
    return self.integrate(self.vel, self.time)


class pose_estimator:
  
  def __init__(self):

    rospy.init_node("pose_estimator")

    self.start_time=time.time()

    self.current_pose=rospy.Publisher("/calypso_sim/pose",pose, queue_size=10)

    self.current_vellocity=rospy.Publisher("/calypso_sim/velocity",pose, queue_size=10)

    self.pose=pose()
    self.velocity=pose()

    self.rate = rospy.Rate(10) 

    self.dvl_x=cord()
    self.dvl_y=cord()
    self.dvl_z=cord()
  
  def dvl_subscriber(self,dvl):
     
    time_elapsed = time.time() - self.start_time
    self.dvl_x.vel.append(dvl.velocity.x)
    self.dvl_y.vel.append(dvl.velocity.y)
    self.dvl_z.vel.append(dvl.velocity.z)
    # self.dvl_x.time.append(time_elapsed)
    # self.dvl_y.time.append(time_elapsed)
    # self.dvl_y.time.append(time_elapsed)
    
    self.velocity.linear_position.x=self.dvl_x.vel[-1]
    self.velocity.linear_position.y=self.dvl_y.vel[-1]
    self.velocity.linear_position.z=self.dvl_z.vel[-1]
    self.pose.linear_position.x=round(self.dvl_x.get_dvl_pose(),3) 
    self.pose.linear_position.y=round(self.dvl_y.get_dvl_pose(),3) 
    self.pose.linear_position.z=round(self.dvl_z.get_dvl_pose(),3) 

    self.current_pose.publish(self.pose)
    self.current_vellocity.publish(self.velocity)

  def Imu_subscriber(self, Imu):
    
    R,P,Y= np.degrees(euler_from_quaternion([Imu.orientation.x,Imu.orientation.y,Imu.orientation.z,Imu.orientation.w]))

    self.velocity.angular_position.x=round(Imu.angular_velocity.x,3)
    self.velocity.angular_position.y=round(Imu.angular_velocity.y,3)
    self.velocity.angular_position.z=round(Imu.angular_velocity.z,3)
    self.pose.angular_position.x=R
    self.pose.angular_position.y=P
    self.pose.angular_position.z=Y

  def start(self):

    dvl=rospy.Subscriber("/calypso_sim/dvl",DVL,self.dvl_subscriber)
    imu=rospy.Subscriber("/calypso_sim/imu/data",Imu, self.Imu_subscriber)

    rospy.spin()

if __name__ == "__main__":

  try:
    p=pose_estimator()
    p.start()
  except Exception as e:
    print(e)