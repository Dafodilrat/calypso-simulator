import rospy
from geometry_msgs.msg import Quaternion

rospy.init_node("headings")

pub=rospy.Publisher("/calypso_sim/heading",Quaternion,queue_size=10)

pose=Quaternion()

pose.x=6
pose.z=3
pose.y=6
pose.w=0
while not rospy.is_shutdown():
  pub.publish(pose)
