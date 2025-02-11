from dsor_msgs.msg import Measurement 
from std_msgs.msg import Float64MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import Float64
import numpy as np
import rospy
import math

def wrap_to_pi(angle):

    wrapped_angle = (angle + math.pi) % (2 * math.pi) - math.pi

    return wrapped_angle

class Ekf:
    def __init__(self):
        rospy.init_node("Ekf_node")
        self.beta = np.deg2rad(0)
        self.alpha=np.deg2rad(30)
        self.theta=np.deg2rad(0)
        self.y1=0
        self.y2=0
        self.Vu=0
        self.Vz=0
        self.Meas = np.array([[0] , [0]],dtype=np.float64)
        self.Yhat = np.array([[0] , [0]],dtype=np.float64)
        self.xhat = np.array([[20] , [0]], dtype=np.float64)
        self.sigma = np.array([[(10*0.05)**2 ,0],[0,(0.0001*0.05)**2]], dtype=np.float64)
        self.L = np.array([[1,0],[0,1]], dtype=np.float64)
        self.Q = np.array([[0.1**2,0],[0,(np.deg2rad(1))**2]],dtype=np.float64)
        self.R = np.array([[0.25,0],[0,0.25]],dtype=np.float64)
        self.covmsg = numpy_msg(Floats)
        self.Norm_Vel = 0
        self.dt = 0.1
        self.now = rospy.Time.now()
        self.prev = 0
        self.initialize_publishers()
        self.initialize_subscribers()
        self.initialize_timers()
        rospy.spin()
    
    def initialize_subscribers(self):
        rospy.Subscriber("/bluerov_heavy0/measurement/position",Measurement,self.altitude_callback)
        rospy.Subscriber("/bluerov_heavy0/measurement/velocity",Measurement,self.velocity_callback)
        rospy.Subscriber("/bluerov_heavy0/measurement/orientation",Measurement,self.orientation_callback)

    def initialize_publishers(self):
        self.pred_pub = rospy.Publisher("Ekf/predict_state",Floats,queue_size=5)
        self.h_h_dot_pub = rospy.Publisher("bluerov_heavy0/bottom_profiling/altitude_altitude_rate",Float64MultiArray,queue_size=5)
        self.cov_pub = rospy.Publisher("Ekf/covarience",Floats,queue_size=5)
        self.xdot_pub = rospy.Publisher("Ekf/xdot",Floats,queue_size=5)
        self.altimeter_pub = rospy.Publisher("Ekf/altimeter1",Float64,queue_size=5)
        self.altimeter2_pub = rospy.Publisher("Ekf/altimeter2",Float64,queue_size=5)
        # self.altitude_rate_pub = rospy.Publisher("/bluerov_heavy0/bottom_profiling/altitude_rate",Float64,queue_size=5)

        

    def initialize_timers(self):
        rospy.Timer(rospy.Duration(0.1),self.timercallback)

    def altitude_callback(self,msg):
        value = msg.value
        if msg.header.frame_id == "bluerov_heavy0/altimeter1_link":# edited xarco of altimeter for changing topic
            self.Meas[0][0] = msg.value[0]
            self.altimeter_pub.publish(Float64(msg.value[0]))
        # print("altitude:",self.Meas[0][0])
        
        if msg.header.frame_id == "bluerov_heavy0/altimeter2_link":
            self.Meas[1][0] = msg.value[0]
            self.altimeter2_pub.publish(Float64(msg.value[0]))


    def orientation_callback(self,msg):
        value = msg.value
        self.theta = value[1]


    
    def velocity_callback(self,msg):
        msg1 = msg
        self.Vu = float(int(msg1.value[0]*1e2))/1e2
        self.Vv = float(int(msg1.value[1]*1e2))/1e2
        self.Vz = float(int(msg1.value[2]*1e2))/1e2
        self.Norm_Vel = (self.Vu**2+self.Vv**2)**0.5
        # print("velocities",self.Vu,self.Vz,self.Vv,self.Norm_Vel)


    def timercallback(self,event):
        self.predict()

    def predict(self):
        self.prev = self.now
        self.now =  rospy.Time.now()
        self.dt = self.now - self.prev
        # print("Dt: ",self.dt.to_sec())
        
        if self.sigma[0][0] == 0 :
            self.sigma[0][0] == (4*0.05)**2
            self.sigma[1][1] == ((0.0001)*0.05)**2  
        
        
        #prediction
        V = self.Norm_Vel
        # self.h = self.xhat[0][0]  
        self.beta = self.xhat[1][0]
        hdot = -self.Vz - np.tan(self.beta)*V
        print("hdot",hdot)
        betadot = 0
        xph = self.xhat[0][0]  + hdot*self.dt.to_sec()
        print("xph,dt:",xph,",",self.dt.to_sec())
        xpbeta = self.beta + betadot*self.dt.to_sec()
        xdot = np.array([[xph],[xpbeta]])
        self.xdot_pub.publish(xdot)
        self.xhat[0][0]=xph
        self.xhat[1][0]=xpbeta
        self.beta = xpbeta
        
        #ekf linearization
        Ahat = np.array([[0,V*((1/np.cos(xpbeta))**2)],[0,0]])
        Cdhat = np.array([[np.cos(self.beta)/np.cos(self.beta-self.theta) , ((-np.sin(self.beta)*np.cos(self.beta-self.theta))+(np.sin(self.beta-self.theta)*np.cos(self.beta))*self.xhat[0][0])/np.cos(self.beta-self.theta)**2],
                            [np.cos(self.beta)/np.cos(self.beta-self.theta-self.alpha) , ((-np.sin(self.beta)*np.cos(self.beta-self.theta-self.alpha))+(np.sin(self.beta-self.theta-self.alpha)*np.cos(self.beta))*self.xhat[0][0])/np.cos(self.beta-self.theta-self.alpha)**2]])
        
        print(" \u03B2 :",self.beta," \u03B8 :",self.theta," heigh:",xph)
        #covariance prediction
        sigmap = np.add(np.linalg.multi_dot([Ahat,self.sigma,Ahat.T]),np.linalg.multi_dot([self.L,self.Q,self.L.T]))
        Sk = np.add(np.linalg.multi_dot([Cdhat,sigmap,Cdhat.T]),self.R)
        Sk_inv = np.linalg.inv(Sk)
        Hk = np.linalg.multi_dot([sigmap,Cdhat.T,Sk_inv])


        #update covariance
        print("\n Ahat:",Ahat,"\n Sk: \n\n",Sk,"\n Cdhat: \n\n",Cdhat,"\n sigmap: \n\n",sigmap, "\n Hk: \n\n",Hk,"\n Sk_inv: \n\n",Sk_inv)
        self.sigma = np.subtract(sigmap,np.linalg.multi_dot([sigmap,Hk.T,Sk_inv,Cdhat,sigmap]))
        covX = np.diag(self.sigma) 
        self.cov_pub.publish(covX)
        
        #predict measurement
        self.Yhat[0][0] = self.xhat[0][0]*np.cos(self.beta)/np.cos(self.beta-self.theta)
        self.Yhat[1][0] = self.xhat[0][0]*np.cos(self.beta)/np.cos(self.beta-self.theta-self.alpha)

        #update states
        print("\n Yhat: \n\n",self.Yhat,"\n Meas: \n\n",self.Meas)
        self.xhat = np.add(self.xhat,np.linalg.multi_dot([Hk,np.subtract(self.Meas,self.Yhat)]))
        xhat_copy = self.xhat
        xhat_copy[1] = wrap_to_pi(self.xhat[1])
        self.pred_pub.publish(xhat_copy)
        print("xhat",self.xhat)
        # self.altitude_rate_pub.publish(Float64(self.xhat[0]))

        msg = Float64MultiArray()
        msg.data = [self.xhat[0], hdot]
        self.h_h_dot_pub.publish(msg)


x = Ekf()



















































# from dsor_msgs.msg import Measurement 
# from rospy.numpy_msg import numpy_msg
# from rospy_tutorials.msg import Floats
# import numpy as np
# import rospy


# class Ekf:
#     def __init__(self):
#         rospy.init_node("Ekf_node")
#         self.beta = np.deg2rad(0)
#         self.h =4
#         self.alpha=np.deg2rad(45)
#         self.theta=np.deg2rad(0)
#         self.y1=0
#         self.y2=0
#         self.Vu=0
#         self.Vz=0
#         self.Meas = np.array([[0] , [0]],dtype=np.float32)
#         self.Yhat = np.array([[0] , [0]],dtype=np.float32)
#         self.xhat = np.array([[0] , [0]], dtype=np.float32)
#         self.sigma = np.array([[0 ,0],[0,0]], dtype=np.float32)
#         self.L = np.array([[1,0],[0,1]], dtype=np.float32)
#         self.Q = np.array([[0.1**2,0],[0,(np.deg2rad(1))**2]],dtype=np.float32)
#         self.R = np.array([[0.5,0],[0,0.5]],dtype=np.float32)
#         self.covmsg = numpy_msg(Floats)
#         self.dt = 0.1
#         self.now = rospy.Time.now()
#         self.prev = 0
#         self.initialize_publishers()
#         self.initialize_subscribers()
#         self.initialize_timers()
#         rospy.spin()
    
#     def initialize_subscribers(self):
#         rospy.Subscriber("/bluerov_heavy0/measurement/position",Measurement,self.altitude_callback)
#         rospy.Subscriber("/bluerov_heavy0/measurement/velocity",Measurement,self.velocity_callback)

#     def initialize_publishers(self):
#         self.pred_pub = rospy.Publisher("Ekf/predict_state",numpy_msg(Floats),queue_size=5)
#         self.cov_pub = rospy.Publisher("Ekf/covarience",numpy_msg(Floats),queue_size=5)
#         self.xdot_pub = rospy.Publisher("Ekf/xdot",numpy_msg(Floats),queue_size=5)

#     def initialize_timers(self):
#         rospy.Timer(rospy.Duration(0.1),self.timercallback)

#     def altitude_callback(self,msg):
#         value = msg.value
#         if msg.header.frame_id == "bluerov_heavy0/altimeter1_link":
#             self.Meas[0][0] = msg.value[0]
#             if self.h == 0 :
#                 self.h = self.Meas[0][0]
#             # print(self.Meas[0][0]/self.Meas[1][0])
#         # print("altitude:",self.Meas[0][0])
        
#         if msg.header.frame_id == "bluerov_heavy0/altimeter2_link":
#             self.Meas[1][0] = msg.value[0]
#             # print(self.Meas[0][0]/self.Meas[1][0])


    
#     def velocity_callback(self,msg):
#         # msg = Measurement(msg)
#         self.Vu = msg.value[0]
#         self.Vz = msg.value[2]

#     def timercallback(self,event):
#         self.predict()

#     def predict(self):
#         self.prev = self.now
#         self.now =  rospy.Time.now()
#         self.dt = self.now - self.prev
#         xp = np.array([[0],[0]])

#         if self.sigma[0][0] == 0:
#             self.sigma[0][0] == (4*0.05)**2
#             self.sigma[1][1] == ((0.0001)*0.05)**2  
        
#         #prediction
#         if self.xhat[0][0] == 0:
#             self.h = self.Meas[0][0]
#         else:
#             self.h = self.xhat[0][0]
#             self.beta = self.xhat[1][0]

#         hdot = -self.Vz - np.tan(self.beta)*self.Vu
#         betadot = 0
#         xph = self.h + hdot*self.dt.to_sec()
#         xpbeta = self.beta + betadot*self.dt.to_sec()
#         xdot = np.array([[xph],[xpbeta]])
#         self.xdot_pub.publish(xdot)
#         xp[0][0]=xph
#         xp[1][0]=xpbeta
#         self.beta=xpbeta

#         #ekf linearization
#         Ahat = np.array([[0,-self.Vu*((1/np.cos(xpbeta))**2)],[0,0]])
#         Cdhat = np.array([[np.cos(self.beta)/np.cos(self.beta-self.theta) , ((-np.sin(self.beta)*np.cos(self.beta-self.theta))+(np.sin(self.beta-self.theta)*np.cos(self.beta))*xp[0][0])/np.cos(self.beta-self.theta)],
#                             [np.cos(self.beta)/np.cos(self.beta-self.theta-self.alpha) , ((-np.sin(self.beta)*np.cos(self.beta-self.theta-self.alpha))+(np.sin(self.beta-self.theta-self.alpha)*np.cos(self.beta))*xp[0][0])/np.cos(self.beta-self.theta-self.alpha)]])
        
#         print(" \u03B2 :",self.beta," \u03B8 :",self.theta," heigh:",xph)
#         #covariance prediction
#         sigmap = np.add(np.linalg.multi_dot([Ahat,self.sigma,Ahat.T]),np.linalg.multi_dot([self.L,self.Q,self.L.T]))
#         Sk = np.add(np.linalg.multi_dot([Cdhat,sigmap,Cdhat.T]),self.R)
#         Sk_inv = np.linalg.inv(Sk)
#         Hk = np.add(np.linalg.multi_dot([sigmap,Cdhat]),Sk_inv)


#         #update covariance
#         print("\n Ahat:",Ahat,"\n Sk: \n\n",Sk,"\n Cdhat: \n\n",Cdhat,"\n sigmap: \n\n",sigmap)
#         self.sigma = np.subtract(self.sigma,np.linalg.multi_dot([sigmap,Hk.T,Sk_inv,Cdhat,sigmap]))
#         covX = np.diag(self.sigma) 
#         self.cov_pub.publish(covX)
        
#         #predict measurement
#         self.Yhat[0][0] = xp[0][0]*np.cos(self.beta)/np.cos(self.beta-self.theta)
#         self.Yhat[1][0] = xp[1][0]*np.cos(self.beta)/np.cos(self.beta-self.theta-self.alpha)

#         #update states
#         self.xhat = np.add(xp,np.linalg.multi_dot([Hk,np.subtract(self.Meas,self.Yhat)]))
#         self.pred_pub.publish(self.xhat)


# x = Ekf()
# print(x.Meas[0],x.Meas[1])