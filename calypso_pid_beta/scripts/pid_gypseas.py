#! /usr/bin/python3
import rospy
from calypso_msgs.msg import gypseas
from sensor_msgs.msg import Imu
from pid_calypso import pid as PID
from calypso_pose_estimator.msg import pose
from scipy.interpolate import interp1d
import time
from geometry_msgs.msg import Quaternion
# from scipy.integrate import trapezoid

class pid_gypseas:
  
  def __init__(self):

    rospy.init_node('gypseas_pid', anonymous=False)


    self.roll=PID()
    self.pitch=PID()
    self.heave=PID()
    self.yaw=PID()

    self.pitch.k=[2,0.5,0.05]
    self.roll.k=[2,0.5,0.05]
    # [100,0.165,800]
    self.heave.k=[10,0,1]
          
    self.throttle1 = 1582
    self.throttle2 = 1582
    self.throttle3 = 1582
    self.throttle4 = 1582

    self.m = interp1d([0, 90],[1582,1900])
    self.n = interp1d([-90, 0], [1500,1582])

    self.rate = rospy.Rate(10)

    self.gypseas_publisher = rospy.Publisher('/rosetta/gypseas', gypseas, queue_size=10)

    self.start_time = time.time()
  
  def gypseas_pid(self):

    # fir testing purpose
    PID_pitch = PID.getPID(self.pitch,True)
    PID_roll = PID.getPID(self.roll,True)
    # PID_heave = PID.getPID(self.heave, False)


    # if we wanna map linear motiion: 
    PID_heave_ = PID.getPID(self.heave, False)
    if PID_heave_>=0:
      PID_heave = self.m(PID_heave_)
    else:
      PID_heave = self.n(PID_heave_)
    # for testing gyro PID, comment the PID_heave part.

    # self.g=gypseas()src/calypso-simulator/calypso_pid_beta/scripts/pid_gypseas.py
    # self.g.t1 = round(self.throttle1 + PID_roll - PID_pitch + PID_heave)
    # self.g.t2 = round(self.throttle2 - PID_roll - PID_pitch + PID_heave)
    # self.g.t3 = round(self.throttle3 - PID_roll + PID_pitch + PID_heave)
    # self.g.t4 = round(self.throttle4 + PID_roll + PID_pitch + PID_heave)

    self.g=gypseas()
    self.g.t1 = round(PID_heave + PID_roll - PID_pitch)
    self.g.t2 = round(PID_heave - PID_roll - PID_pitch)
    self.g.t3 = round(PID_heave - PID_roll + PID_pitch)
    self.g.t4 = round(PID_heave + PID_roll + PID_pitch)

    print("---------------------------------------------------------------")

    print("current_heave :{} targeted_heave:{}".format(self.heave.current_position,self.heave.final_position))
    
    print("PID-roll : ", PID_roll ," PID-pitch : ",PID_pitch, " PID-heave : ", PID_heave)

    print("g1 :{}, g2 :{}, g3 :{}, g4 :{}".format(self.g.t1,self.g.t2,self.g.t3,self.g.t4))

    self.gypseas_publisher.publish(self.g)
    self.rate.sleep()
  
  def Heading_subscriber(self,pose):

    self.heave.final_position = pose.z

  def get_current_pose(self, pose):
    
    time_elapsed =  time.time()-self.start_time
    # self.pitch.time.append(time_elapsed)
    # self.roll.time.append(time_elapsed)
    # self.heave.time.append(time_elapsed)

    self.heave.current_position = pose.linear_position.z
    self.roll.current_position = pose.angular_position.x
    self.pitch.current_position = pose.angular_position.y

    self.gypseas_pid()

  def get_current_velocity(self,velocity):

    self.roll.curent_velocity=velocity.angular_position.x
    self.pitch.curent_velocity=velocity.angular_position.y

  def getgyp(self, gypseas):
    self.throttle1 = gypseas.t1
    self.throttle2 = gypseas.t2
    self.throttle3 = gypseas.t3
    self.throttle4 = gypseas.t4

  
  def start(self):
      
    #gypseas_subscriber = rospy.Subscriber('/rosetta/gypseas',gypseas, self.getgyp)

    pose_subscriber = rospy.Subscriber('/calypso_sim/pose', pose , self.get_current_pose)

    velocity_subscriber = rospy.Subscriber('/calypso_sim/velocity',pose,self.get_current_velocity)

    CORD=rospy.Subscriber("/calypso_sim/heading",Quaternion, self.Heading_subscriber)

    rospy.spin()

if __name__=='__main__':
  try:
      x = pid_gypseas()
      x.start()
  except rospy.ROSInterruptException:
      pass