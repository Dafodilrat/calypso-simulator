import numpy as np
from scipy.integrate import trapezoid
from tf.transformations import euler_from_quaternion


class pid:

    def __init__ (self):
      
        self.k=[]
        self.pid_i=0
        self.error=[]
        self.time=[]
        self.prev_vel=0
        self.current_position=0
        self.current_velocity=0
        self.final_position=0
    
    def integrate(self, y, x):
        try:
            return trapezoid(y, x)
        except Exception as e:
            print(len(y),len(x))
            print("Sync failure !!")
    
    def getPID(self,feed_forward=False):

        kp=self.k[0];ki=self.k[1];kd=self.k[2]
        pid_i=0
        pid_p=0
        pid_d=0
        
        current=self.current_position
        error = self.final_position - current
        
        # this 'self.value' will be the ros subscriber cam feed - the distance that is left to travel.
        # error = self.final_position
        self.error.append(error)

        pid_i = self.integrate(self.error, self.time)
        
        pid_p = kp*error

        if(feed_forward):
            feedforward = self.current_velocity - self.prev_vel
        else:
            feedforward=0
        
        try:
            pid_d = kd*(error[-1]-self.error[-2]) 
        except:
            pass

        if pid_i>max(90-pid_p-pid_d, 0):
            pid_i = max(90-pid_p-pid_d,0)
        
        elif pid_i<min(-90-pid_i-pid_d, 0):
            pid_i = min(-90-pid_p-pid_d,0)

        pid_i_final = ki*pid_i

        time_elapsed=self.time[-1]

        if time_elapsed>0:
            PID = pid_p + pid_i_final + pid_d + feedforward/time_elapsed
        else:
            PID = pid_p + pid_i_final + pid_d
        self.prev_vel = self.current_velocity
        
        if(PID > 90):
            PID=90
        if(PID < -90):
            PID=-90
        
        return PID