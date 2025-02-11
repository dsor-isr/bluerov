import rospy 
from rospy_tutorials import Floats
from rospy.numpy_msg import numpy_msg
import numpy as np
from auv_msgs.msg import BodyForceRequest
from threading import Lock

class AltitudeControl:
    def __init__(self):
        rospy.init_node("Altitude_Control")
        self.href
        self.h
        self.e 
        self.G = np.array([0,0,0])
        self.lock = Lock()
        self.Tz = 0
        self.hdot = 0
        self.hdotp = 0
        self.now = 0
        self.prev = rospy.Time.now()
        self.set_filter_gain(np.array([1,1,1]))
        self.init_publishers()
        self.init_subscribers()
        self.init_timers()

    def init_subscribers(self):
        rospy.Subscriber("/Ekf/predict_state",numpy_msg(Floats),self.state1_callback)
        rospy.Subscriber("/Ekf/xdot",numpy_msg(Floats),self.state2_callback)

    def init_publishers(self):
        self.thrust_pub = rospy.Publisher("/bluerov_heavy0/controls/static_thruster_allocation",BodyForceRequest,queue_size=5)
    
    def init_timers(self):
        rospy.Timer(rospy.Duration(0.1),self.timer_callback)
    
    def check_filter_init(self):
        if np.sum(self.G) and self.h :
            return 1
        else:
            return 0
    
    def set_filter_gain(self,gain):
        self.G = gain

    def state1_callback(self,msg):
        data = np.array(msg.value)
        self.h = data[0][0]

    def state2_callback(self,msg):
        data = np.array(msg.value)
        self.hdot = data[0][0]

    def timer_callback(self,event):
        if self.check_filter_init():    
            self.e = self.h-self.href
            self.prev = self.now
            self.now = rospy.Time.now()
            dt = self.now.to_sec() - self.prev.to_sec()
            dhdotbydt = (self.hdot-self.hdotp)/dt
            int_arg = self.G(1)*dhdotbydt + self.G(2)*self.hdot + self.G(3)*self.e
            self.Tz = self.Tz + int_arg*dt
            msg = BodyForceRequest()
            msg.wrench.force[2] = self.Tz
            self.thrust_pub(msg)

