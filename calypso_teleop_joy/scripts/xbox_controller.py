
import rospy
from calypso_msgs.msg import gypseas
from calypso_msgs.msg import dolphins
from scipy.interpolate import interp1d
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Joy

class control:
	
	def __init__(self):

		self.node=rospy.init_node("controller")

		self.publisher_gypseas = rospy.Publisher("/rosetta/gypseas", gypseas, queue_size=100)
		
		self.publisher_dolphins = rospy.Publisher("/rosetta/dolphins", dolphins, queue_size=100)

		self.rate = rospy.Rate(200)
		self.stable_gypseas = 1584
		self.stable_dolphins=1650
		self.gypseas_min=1520
		self.dolphins_min=1530

		self.thrust_up_gypseas = interp1d([0, 1],[self.stable_gypseas, 1800]) 
		self.thrust_down_gypseas=interp1d([-1, 0],[self.gypseas_min, self.stable_gypseas]) 

		self.thrust_up_dolphins = interp1d([0, 1],[self.stable_dolphins, 1800])
		self.thrust_down_dolphins = interp1d([-1, 0],[self.dolphins_min, self.stable_dolphins])

		self.is_locked = False
		self.g = gypseas()
		self.d = dolphins()
		self.q = Quaternion()
	#def load_mappings(self, ns):
        	#axes_remap = rospy.get_param(ns + "/axes", [])
        	#rospy.loginfo("loaded remapping: %d buttons, %d axes" % (len(btn_remap), len(axes_remap)))
        	#return {"buttons": btn_remap, "axes": axes_remap}
	def f(self,x):

		if x<0:
			x=self.stable_dolphins
		
		if x>260:
			x=260
		
		return x
	
	
	def JoyCallback(self, msg):
		

		self._deadzone = 0.05
		
		self.publisher_dolphins.publish(self.d)
		if rospy.has_param('~deadzone'):
			self._deadzone = float(rospy.get_param('~deadzone'))
		
		if msg is not None:

			################## GYPSEAS ####################

			if(msg.buttons[2]== 1):
				self.is_locked= not (self.is_locked)
				#print(self.is_locked)
			
			if self.is_locked:

				self.g.t1=self.g.t2=self.g.t3=self.g.t4=self.stable_gypseas
			
			else :

				if (msg.axes[1] > 0 and abs(msg.axes[1]) > self._deadzone ):
						self.g.t1=self.g.t2=self.g.t3=self.g.t4 = int(self.thrust_up_gypseas(msg.axes[1]))

				# if (msg.axes[1] == 0 and abs(msg.axes[1]) > self._deadzone ):
				# 		self.g.t1=self.g.t2=self.g.t3=self.g.t4 = self.stable_gypseas	
				
				if (msg.axes[1] < 0 and abs(msg.axes[1]) > self._deadzone):
						self.g.t1 = self.g.t2=self.g.t3=self.g.t4 = int(self.thrust_down_gypseas(msg.axes[1]))

				if(msg.buttons[7]== 1 and -self._deadzone<msg.axes[4]<self._deadzone):
					self.g.t1 = self.g.t2=self.g.t3 =self.g.t4=0

			self.publisher_gypseas.publish(self.g)
				
			################## DOLPHINS ####################
			self.d.d1 = self.d.d2=self.d.d3 =self.d.d4=0
			
			#### forward ####
			if msg.axes[4] > 0 and abs(msg.axes[4]) > self._deadzone:
					self.d.d3 = self.d.d4 = int(self.thrust_up_dolphins(msg.axes[4]))
			
			#### backward ####
			elif msg.axes[4] < 0 and abs(msg.axes[4]) > self._deadzone:
					self.d.d1 = self.d.d2 = int(self.thrust_down_dolphins(msg.axes[4]))

			#### yaw left horizeontal right stick +ve ####
			if(msg.axes[0]>0 and abs(msg.axes[3]) > self._deadzone):
					
					self.d.d1 = self.d.d3 = int(self.thrust_up_dolphins(msg.axes[0]))
					self.d.d2=self.d.d4 = 0

			#### yaw right horizontal right stick -ve ####
			elif(msg.axes[0]<0 and abs(msg.axes[3]) > self._deadzone):
					
				self.d.d2=self.d.d4 = int(self.thrust_down_dolphins(msg.axes[0]))
				self.d.d1=self.d.d3 = 0
			
			self.publisher_dolphins.publish(self.d)
			
		else:
			print("no connection") 
		
	
	def start(self):

		rospy.Subscriber('/joy', Joy, self.JoyCallback)
		rospy.spin()
               		 
if __name__=='__main__':
    try:
        x = control()
        x.start()
    except rospy.ROSInterruptException:
        pass

		
		
