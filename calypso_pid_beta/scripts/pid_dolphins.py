#! /usr/bin/python3
import rospy
from calypso_msgs.msg import dolphins
from sensor_msgs.msg import Imu
from pid_calypso import pid as PID
from calypso_pose_estimator.msg import pose
import time
import math 
from scipy.interpolate import interp1d

class pid_dolphins:

  def __init__(self):

    rospy.init_node('dolphins_pid', anonymous=False)

    self.time_lapsed = 0

    self.yaw=PID()
    self.surge=PID()
    self.sway=PID()

    self.yaw.k=[5,0.5,0.05]
    self.surge.k=[1,0.05,0.05]
    self.sway.k=[1,0.05,0.05]

    self.dolphins_interp = interp1d([-270,270],[1530,1800])
    self.base_thrust=1540

    self.rate = rospy.Rate(10)
    self.dolphins_publisher = rospy.Publisher('/rosetta/dolphins', dolphins, queue_size=10)
    # self.x = 2
    # self.y = 2

    self.start_time = time.time()
  
  def dolphins_pid(self):

    CORD=rospy.Subscriber("calypso_sim/heading",Quaternion,self.Heading_subscriber)
    POSE=rospy.Subscriber("calypso_sim/pose",Quaternion,self.Pose_subscriber)
    d=dolphins()

    pid_surge=self.surge.getPID(False)
    pid_sway=self.sway.getPID(False)
    pid_yaw=self.yaw.getPID(True)

    thruster_1=(self.base_thrust+pid_yaw-pid_surge+pid_sway)
    thruster_2=(self.base_thrust-pid_yaw-pid_surge-pid_sway)
    thruster_3=(self.base_thrust+pid_yaw+pid_surge-pid_sway)
    thruster_4=(self.base_thrust-pid_yaw-pid_surge+pid_sway)

    d.d1=round(thruster_1)
    d.d2=round(thruster_2)
    d.d3=round(thruster_3)
    d.d4=round(thruster_4) 

    print("surge:{} sway:{} yaw:{}".format(pid_surge,pid_sway,pid_yaw))   

    self.dolphins_publisher.publish(d)
    self.rate.sleep()

  def Heading_subscriber(self,pose_next):
    
    if pose_next.w==0:
      self.yaw.final=math.atan2(pose_next.y,pose_next.x)
    elif pose_next.x==0:
      self.yaw.final=0
    else :
      self.yaw.final=pose_next.w

    self.sway.final=pose_next.y
    self.surge.final=pose_next.x
  
  def Pose_subscriber(self,pose_curr):

    self.sway.current_position=pose_curr.y
    self.surge.current_position=pose_curr.x

  def Imu_subscriber(self, Imu):

    time_elapsed = time.time()-self.start_time

    self.surge.time.append(time_elapsed)
    self.sway.time.append(time_elapsed)
    self.yaw.time.append(time_elapsed)
  
    self.yaw.current_vel= Imu.angular_velocity.z
    x,y,self.yaw.current_position= PID.convert(Imu.orientation.x,Imu.orientation.y,Imu.orientation.z,Imu.orientation.w)

    self.dolphins_pid()

  def start(self):

    try:
      IMU = rospy.Subscriber("/calypso_sim/imu/data", Imu, self.Imu_subscriber)
      rospy.spin()
    
    except :
      print("ros shut down !!!")
  
if __name__=='__main__':

  try:
      x = pid_dolphins()
      x.start()
  except rospy.ROSInterruptException:
      pass