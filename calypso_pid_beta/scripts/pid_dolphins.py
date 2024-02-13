#! /usr/bin/python3
import rospy
from calypso_msgs.msg import dolphins
from pid_calypso import pid as PID
from calypso_pose_estimator.msg import pose
import time
import math 
from scipy.interpolate import interp1d
from geometry_msgs.msg import Quaternion
from math import atan2

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

    self.dolphins_interp = interp1d([-270,270],[1525,1800])
    self.yaw_interp= interp1d([-179,-1],[181,359])
    self.base_thrust=1525

    self.rate = rospy.Rate(10)

    self.dolphins_publisher = rospy.Publisher('/rosetta/dolphins', dolphins, queue_size=10)

    self.start_time = time.time()
  
  def dolphins_pid(self):

    pid_surge=self.surge.getPID(False)
    pid_sway=self.sway.getPID(False)
    # pid_yaw=self.yaw.getPID(True)
    pid_yaw=0

    thruster_1=(self.base_thrust+pid_yaw-pid_surge+pid_sway)
    thruster_2=(self.base_thrust-pid_yaw-pid_surge-pid_sway)
    thruster_3=(self.base_thrust+pid_yaw+pid_surge-pid_sway)
    thruster_4=(self.base_thrust-pid_yaw+pid_surge+pid_sway)
    
    d=dolphins()

    d.d1=round(thruster_1)
    d.d2=round(thruster_2)
    d.d3=round(thruster_3)
    d.d4=round(thruster_4) 
    
    print("current_y :{} current_x:{}".format(self.sway.current_position,self.surge.current_position))
    print("final_y :{} final_x:{}".format(self.sway.final_position,self.surge.final_position))
    print("pid_sway :{} pid_surge :{}".format(pid_sway,pid_surge))
    print("d1 :{}, d2 :{}, d3 :{}, d4 :{}".format(d.d1,d.d2,d.d3,d.d4))

    self.dolphins_publisher.publish(d)
    
    self.rate.sleep()

  def Heading_subscriber(self,pose_next):
    
    self.sway.final_position=pose_next.y
    self.surge.final_position=pose_next.x
    
    # if pose_next.x==0:
    #   self.yaw.final=0
    # else:
    #   self.yaw.final=self.yaw_interp(atan2(pose_next.y,pose_next.x))
  
  def get_current_pose(self,pose):

    self.sway.current_position=pose.linear_position.y
    self.surge.current_position=pose.linear_position.x
    
    # if pose.linear_position.x==0:
    #   self.yaw.current_position=0
    # else:
    #   try:
    #     self.yaw.current_position=self.yaw_interp(atan2(pose.linear_position.y,pose.linear_position.x))
    #   except Exception as e:
    #     print(atan2(pose.linear_position.y,pose.linear_position.x))

    self.dolphins_pid()

  def get_current_velocity(self,velocity):
  
    self.yaw.current_velocity= velocity.angular_position.z

  def start(self):

    try:
    
      CORD=rospy.Subscriber("calypso_sim/heading",Quaternion,self.Heading_subscriber)

      POSE=rospy.Subscriber("calypso_sim/pose",pose,self.get_current_pose)
      d=dolphins()
      
      VEL=rospy.Subscriber("calypso_sim/velocity",pose,self.get_current_velocity)

      rospy.spin()
    
    except :
      print("ros shut down !!!")
  
if __name__=='__main__':

  try:
      x = pid_dolphins()
      x.start()
  except rospy.ROSInterruptException:
      pass