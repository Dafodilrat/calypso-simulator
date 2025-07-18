#!/bin/env python3
import rospy
from geometry_msgs.msg import Quaternion

def publish_pose():
  
    pose=Quaternion()

    pose.x=6
    pose.y=0
    pose.z=6
    pose.w=0
    posepub.publish(pose)

if __name__=="__main__":
  
  rospy.init_node('posepub',anonymous=False)
  posepub=rospy.Publisher("/calypso_sim/heading",Quaternion,queue_size=10)    
  rate=rospy.Rate(10)
  
  while not rospy.is_shutdown():

    publish_pose()
    rate.sleep()
