#!/usr/bin/env python

""" 
Developers: DSOR Team -> @tecnico.ulisboa.pt Instituto Superior Tecnico
"""
import rospy
# from python_counter_algorithms.PythonCounterAlgorithm import count_function
from bottom_following_algorithms.BottomFollowing import BottomFollowing, OutlierRejection
from std_msgs.msg import Int32, Bool

class BottomFollowingNode():
    def __init__(self):
        """
        Constructor for ros node
        """

        """
        @.@ Init node
        """
        rospy.init_node('python_counter_node')

        
        """
        @.@ Handy Variables
        # Declare here some variables you might think usefull -> example: self.fiic = true
        """
        

        """
        @.@ Dirty work of declaring subscribers, publishers and load parameters 
        """
        self.loadParams()
        self.initializeSubscribers()
        self.initializePublishers()
        self.initializeTimer()
        #self.initializeServices()
        
        self.bottom_follower = BottomFollowing()
        self.ouliter_rejector = OutlierRejection()
        

        

    """
    @.@ Member Helper function to set up parameters; 
    """
    def loadParams(self):
        self.node_frequency = rospy.get_param('~node_frequency')
        # rospy.loginfo('node_frequency acquired: ' + str(self.node_frequency))
        self.num = rospy.get_param('~start_num')
        self.pause = False


    """
    @.@ Member Helper function to set up subscribers; 
    """
    def initializeSubscribers(self):
        rospy.loginfo('Initializing Subscribers for BottomFollowingNode')
        # Altimeter sub
        rospy.Subscriber(rospy.get_param('~topics/subscribers/pause'), Bool, self.set_pause)
        # DVL / ALtimeter2 sub
        rospy.Subscriber(rospy.get_param('~topics/subscribers/pause'), Bool, self.set_pause)
    
    """
    @.@ Member Helper function to set up publishers; 
    """
    def initializePublishers(self):
        rospy.loginfo('Initializing Publishers for BottomFollowingNode')
        self.surge_ref_pub = rospy.Publisher(rospy.get_param('~topics/publishers/surge_ref'), Int32, queue_size=10)
        self.sway_ref_pub = rospy.Publisher(rospy.get_param('~topics/publishers/sway_ref'), Int32, queue_size=10)
        self.heave_ref_pub = rospy.Publisher(rospy.get_param('~topics/publishers/heave_ref'), Int32, queue_size=10)
        

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


    """
    @.@ Timer iter callback. Where the magic should happen
    """
    def timerIterCallback(self, event=None):
        pass



def main():

    bottom_following_node = BottomFollowingNode()

    # +.+ Going into spin; let the callbacks do all the magic 
    rospy.spin()

if __name__ == '__main__':
    main()
