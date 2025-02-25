#!/usr/bin/env python

""" 
Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico
"""
import rospy
from bottom_following_algorithms.BottomFollowing import BottomFollowing
from bottom_following_algorithms.OutlierRejection import OutlierRejection
from std_msgs.msg import Float64, Bool, Int8, Empty
from geometry_msgs.msg import Vector3
from rospy_tutorials.msg import Floats
from math import pi, sqrt, acos, atan2
from auv_msgs.msg import NavigationStatus
from dsor_msgs.msg import Measurement
from a50_dvl.msg import DVL
from sensor_msgs.msg import Range

def wrap_to_pi(angle):
    return (angle + pi) % (2 * pi) - pi


class BottomFollowingNode():
    def __init__(self):
        """
        Constructor for ros node
        """

        """
        @.@ Init node
        """
        rospy.init_node('bottom_following_node')

        
        """
        @.@ Handy Variables
        # Declare here some variables you might think usefull -> example: self.fiic = true
        """
        self.initialized  = False
        self.h1 = None
        self.h2 = None
        self.V = None
        self.alpha = None
        self.simulation=True
        self.flag=0
        self.v_ref=0
        self.v_ref_t = None
        self.d_ref = None
        self.d_ref_t =None
        self.u_max =0.5
        

        """
        @.@ Dirty work of declaring subscribers, publishers and load parameters 
        """
        self.loadParams()
        self.initializeSubscribers()
        self.initializePublishers()
        self.initializeTimer()
        #self.initializeServices()
        self.bottom_follower = None
        self.ouliter_rejector = None
        self.last_time = rospy.Time.now()
        

        
        

        

    """
    @.@ Member Helper function to set up parameters; 
    """
    def loadParams(self):
        self.node_frequency = rospy.get_param('~node_frequency')
        self.alpha =pi/180*rospy.get_param('~alpha', 27)
        
        # rospy.loginfo('node_frequency acquired: ' + str(self.node_frequency))
        self.num = rospy.get_param('~start_num')
        self.pause = False


    """
    @.@ Member Helper function to set up subscribers; 
    """
    def initializeSubscribers(self):
        rospy.loginfo('Initializing Subscribers for BottomFollowingNode')
        # Altimeter sub
        rospy.Subscriber(rospy.get_param('~/topics/subscribers/altimeter', "/bluerov_heavy0/measurement/position"), Measurement, self.altitude_callback)
        # DVL / ALtimeter2 sub
        rospy.Subscriber(rospy.get_param('~/topics/subscribers/dvl_beams', '/bluerov_heavy0/drivers/dvl/data'), DVL, self.dvl_range_callback)
        rospy.Subscriber(rospy.get_param('~/topics/subscribers/dvl_beam_sim', '/bluerov_heavy0/dvl_sonar0'), Range, self.dvl_range_sim_callback)
        # velocity sub
        rospy.Subscriber(rospy.get_param('~/topics/subscribers/speed', '/bluerov_heavy0/measurement/velocity'), Measurement, self.speed_callback)
        rospy.Subscriber(rospy.get_param('~/topics/subscribers/orientation', '/bluerov_heavy0/measurement/orientation'), Measurement, self.orientation_callback)
        rospy.Subscriber(rospy.get_param('~/topics/subscribers/enable', '/bluerov_heavy0/bottom_following/enable'), Empty, self.enable_controller_callback)
        rospy.Subscriber(rospy.get_param('~/topics/subscribers/speed_ref', '/bluerov_heavy0/bottom_following/ref/speed'), Float64, self.v_ref_callback)
        rospy.Subscriber(rospy.get_param('~/topics/subscribers/distance_ref', '/bluerov_heavy0/bottom_following/ref/distance'), Float64, self.d_ref_callback)
        
    
    """
    @.@ Member Helper function to set up publishers; 
    """
    def initializePublishers(self):
        rospy.loginfo('Initializing Publishers for BottomFollowingNode')
        self.surge_ref_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/surge','/bluerov_heavy0/ref/surge'), Float64, queue_size=10)
        self.sway_ref_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/sway','/bluerov_heavy0/ref/sway'), Float64, queue_size=10)
        self.heave_ref_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/heave', '/bluerov_heavy0/ref/heave'), Float64, queue_size=10)
        self.flag_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/flag', '/bluerov_heavy0/Flag'), Int8, queue_size=10)
        self.D_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/D', '/bluerov_heavy0/bottom_following/D'), Vector3, queue_size=10)
        self.D_dot_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/D_dot', '/bluerov_heavy0/bottom_following/D_dot'), Vector3, queue_size=10)
        self.debug_pub = rospy.Publisher(rospy.get_param('~/topics/publishers/debug', '/bluerov_heavy0/bottom_following/debug'), Floats, queue_size=10)
        

    """
    @.@ Member helper function to set up the timer
    """
    def initializeTimer(self):
        self.timer = rospy.Timer(rospy.Duration(1.0/self.node_frequency),self.timerIterCallback)


    """
    @.@ Member helper function to shutdown timer;
    """
    def shutdownTimer(self):
        self.timer.shutdown()

    def altitude_callback(self,msg):
        if "altimeter" in msg.header.frame_id:
            self.h1 = msg.value[0]
           
    def speed_callback(self, msg):
        self.V = [msg.value[0], msg.value[1], msg.value[2]] 
        
    def orientation_callback(self, msg):
        self.pitch = wrap_to_pi( msg.value[1]/180*pi) 
    
    def dvl_range_callback(self, msg):
        pass#self.h2 = (msg.beams[0].distance + msg.beams[1].distance)/2
    
    def dvl_range_sim_callback(self, msg):
        self.h2 = msg.range
    
    def enable_controller_callback(self, msg):
        self.flag_pub.publish(Int8(12))
        self.flag=12
        
    def flag_callback(self, msg):
        self.flag = msg.data
    
    def v_ref_callback(self, msg):
        self.v_ref = msg.data
        self.v_ref_t = rospy.Time.now()
        
    def d_ref_callback(self, msg):
        self.d_ref = msg.data
        self.d_ref_t = rospy.Time.now()
        

    """
    @.@ Timer iter callback. Where the magic should happen
    """
    def timerIterCallback(self, event=None):
        t_now = rospy.Time.now()
        Dt = (t_now - self.last_time).to_sec()
        self.last_time = t_now
        
        if not self.initialized:
            if self.h1 is not None and self.h2 is not None:
                self.bottom_follower = BottomFollowing([self.h1, self.h2], self.alpha)
                #elf.ouliter_rejector = OutlierRejection()
                self.initialized = True
                self.h1 = None
                self.h2 = None
                return
            
        if self.h1 is not None and self.h2 is not None:
            
            # Run the estimator
            self.bottom_follower.compute([[self.V[0]],[self.V[2]]], [self.h1, self.h2], self.pitch, Dt)
            self.D_pub.publish(Vector3(self.bottom_follower.xhat[0], sqrt(self.bottom_follower.xhat[0]**2+ self.bottom_follower.xhat[1]**2), self.bottom_follower.xhat[1]))
            self.D_dot_pub.publish(Vector3(self.bottom_follower.xhat[2], 0, self.bottom_follower.xhat[3]))
            
            
            # Run the controller
            # check if distance reference is valid
            if self.d_ref is not None and self.d_ref_t is not  None:
                if (t_now - self.d_ref_t).to_sec() < 0.2:
                    
                    # check if lateral speed reference is valid
                    if self.v_ref_t != None:
                        if (t_now - self.v_ref_t).to_sec() > 0.2:
                            self.v_ref =0
                
                    V_ref = self.bottom_follower.controller(self.d_ref, self.v_ref, self.u_max)
                    self.surge_ref_pub.publish(Float64(V_ref[0]))
                    self.sway_ref_pub.publish(Float64(0))
                    self.heave_ref_pub.publish(Float64(V_ref[1]))
            
            # debugging message
            try:
                instant_alpha = acos(self.h1/self.h2)*180/pi
                
            except:
                instant_alpha=0.0
            distance =  sqrt(self.bottom_follower.xhat[0]**2+self.bottom_follower.xhat[1]**2)
            self.debug_pub.publish(Floats([self.h1, self.h2, instant_alpha, self.bottom_follower.e, distance, 180/pi*atan2(self.bottom_follower.xhat[0], self.bottom_follower.xhat[1])]))




def main():

    bottom_following_node = BottomFollowingNode()

    # +.+ Going into spin; let the callbacks do all the magic 
    rospy.spin()

if __name__ == '__main__':
    main()
