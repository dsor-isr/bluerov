 #!/usr/bin/env python

import cv2
import gi
import numpy as np
import threading 
import rospy
import csv
import time
from dsor_msgs.msg import Measurement
from std_msgs.msg import Bool
from auv_msgs.msg import NavigationStatus
from itertools import zip_longest

from std_msgs.msg import Float64


import pandas as pd
import matplotlib.pyplot as plt
import cv2
from PIL import Image
import time
from skimage import data
from skimage import io,filters,feature
import datetime
import math
from skimage.filters import roberts,sobel,scharr , prewitt,farid
from skimage.feature import canny
from skimage.feature import corner_harris
from skimage.feature import corner_peaks  

df =[{'altitude':0},{'timestamp':0},{'area':0}]
df = pd.DataFrame(df)



gi.require_version('Gst', '1.0')
from gi.repository import Gst
init = 0
area= 0 
val =0 
def getyaw(msg):
        valueyaw =  msg.data
        # print("current heading ", valueyaw)
# def getsub(msg):
#         datasub = msg.data
#         if(ang <= 180 and ang>= 135):
#               val = 180 - ang  
#               val = val * -1
#               valn = val
#         elif(ang >= 90 and ang <= 135):
#                 val = ang -90
#                 valn = val
#         else:
#                 val = ang
#                 val = val * -1
#         if(datasub == 1):
#         #Put text over the plotted contour for the angle 
#             cv2.putText(
#                         org_img,
#                         f"rope {round(val)}",
#                         org,
#                         cv2.FONT_HERSHEY_SIMPLEX,
#                         0.5,
#                         (0, 0, 255),
#                         2,
#                         cv2.LINE_AA,
#                     )
#         return datasub
# print(init)
# rospy.Subscriber('/bluerov_heavy0/is_submerged',Bool, getsub)
#Check if rov is submerged or not 
rospy.init_node('nodesubmerged','listner','orient')
 
def nothing(x):
        pass

cv2.namedWindow("Trackbars")

cv2.createTrackbar("L - H", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - S", "Trackbars", 0, 255, nothing)
cv2.createTrackbar("L - V", "Trackbars", 200, 255, nothing)
cv2.createTrackbar("U - H", "Trackbars", 255, 255, nothing)
cv2.createTrackbar("U - S", "Trackbars", 50, 255, nothing)
cv2.createTrackbar("U - V", "Trackbars", 255, 255, nothing)


cv2.setTrackbarPos("L - H", "Trackbars",5)
cv2.setTrackbarPos("L - S", "Trackbars",50)
cv2.setTrackbarPos("L - V", "Trackbars",40)
cv2.setTrackbarPos("U - H", "Trackbars",35)
cv2.setTrackbarPos("U - S", "Trackbars",200)
cv2.setTrackbarPos("U - V", "Trackbars",255)

def getalt(msg):
    # Extract the timestamp and values
       value = 0
       ts = 0 
       timestamp = msg.header.stamp.secs
       frame_id = msg.header.frame_id
       value = msg.value
       noise = msg.noise
       if (frame_id=="bluerov_heavy0_altimeter"):
        #  print(f"Value: {value}")
        #  print(f"Timestamp: {timestamp}")
        #  print("alt",value)

        #Condition for the null values
            if (value!=0):
                if (area!=0):
                    
                    ts = time.time()
                    #  print("time:",ts)
                    #  print("area",area)
                    data = [{value,area}]
                    df= pd.DataFrame(data)          
                    df.to_csv('output.csv', mode='a', index=True, header=False)
                    # print(data)
                    y= -0.01541013 * area +3.77408217 
                    #Regression eqiation for a specific rope to depth 
                    # print(y)
                    # print("true value" ,value)


            
 
# def getarea(area):
#     print(area)       
    
# def getangle(msg, val):

#             angl = msg.orientation.z    
#             # print(angl)
#             if(ang <= 180 and ang>= 135):
#               val = 180 - ang  
#               val = val * -1
#             elif(ang >= 90 and ang <= 135):
#                 val = ang -90
#             else:
#                 val = ang
#                 val = val * -1
            



#             pub = rospy.Publisher('/bluerov_heavy0/ref/yaw', Float64)
#             #Add val calculated as correction to the current heading 
#             pub.publish(angl + val )
#             print("val",val,"angle", angl, "target", angl+ val)
#             # print("angle", val)
#             return val



def calculate_cross_track_error(x, y, x_point):
    # Define the coordinates of the two endpoints of the line
    xs = x[0]
    ys = 460
    xe = x[1]
    ye = 420

    # Calculate the slope angle (beta) of the line
    beta = math.atan2(ye - ys, xe - xs)

    # Calculate the cross-track error (derr) for the point (x[0], 460)
    x_err = 240 - x_point
    y_err = 480 - ys  # Note: Assuming the point has a fixed y-coordinate of 460
    cosine = (x[1] - x[0])/ math.sqrt((x[1] -x[0])**2 + (460-420)**2)
    # Project the point onto the line
    gam = (x_err * math.cos(-beta)) - (y_err * math.sin(-beta))
    YNew = (x_err * math.sin(-beta)) + (y_err * math.cos(-beta))
    derr = YNew  # Cross-track error
    print(  cosine)
    return derr


def getangle2centre(msg, x):

            angl = msg.orientation.z    

             
            x1, y1 = x[0],420
            x2, y2 = 240,480  # Replace with the coordinates of your second point
            distance = abs(x1 - x2) + abs(y1 - y2)
            angle = math.atan2((y1 -y2),(x1-x2))
            np.rad2deg((angle))
           

            val = np.rad2deg((angle))
            # print(ang)
            if(angle <= 180 and angle>= 135):
                val = 180 - angle  
                val = val * -1
                valn = val
            elif(angle >= 90 and angle <= 135):
                    val = angle -90
                    valn = val
            else:
                    val = angle
                    val = val * -1
            
            pub = rospy.Publisher('/bluerov_heavy0/ref/yaw', Float64, queue_size=2)
            #Add val calculated as correction to the current heading 
            pub.publish(angl + val  )
             
 
            
            # # x1 = x[0]
            # # y1 = 460
            # # x2a = x[-1]
            # # y2a = 0
            # # xa = (x[-1] + x2a )/ 2
            # # # xa = x2a
            # # cosine = ((x[-1] - x[0])/ math.sqrt((x[-1] -x[0])**2 + (20-440)**2))
            # cosine = (math.sqrt((x2 - x1)**2)/ math.sqrt((x2 -x1)**2 + 400))
            ks = 0.001
            dist = math.sqrt((x2 -x1)**2 + (y2-y1)**2)
            pub_sway = rospy.Publisher('/bluerov_heavy0/ref/sway', Float64,queue_size=10)
            pub_depth = rospy.Publisher('/bluerov_heavy0/ref/depth', Float64,queue_size=4)
            # if x[0] > 240 :
            #      pub_sway.publish( calculate_cross_track_error(x , y , 240) *ks * -1)
            #      print(( calculate_cross_track_error(x , y , 240) *ks * -1)) 
            #      print("left")
            # if x[0] < 240 :
            #       pub_sway.publish( calculate_cross_track_error(x , y , 240) *ks)
            #       print(( calculate_cross_track_error(x , y , 240) *ks * 1))
            #       print("right")


             
            # #  #Add val calculated as correction to the current heading 
            depth = 1.5
            pub_depth.publish(depth)
            # # print(distance)
            # print(distance*cosine)
          
            calculate_cross_track_error(x , y , 240)  
            # pub_sway.publish( calculate_cross_track_error(x , y , 240) *ks)



            # print("angle", val)
            return val
  
  
 
# rospy.init_node('listener', anonymous=True)

# def subscriber(area):
#     print("area",area)
   

class Video():
    """BlueRov video capture class constructor

    Attributes:
        port (int): Video UDP port
        video_codec (string): Source h264 parser
        video_decode (string): Transform YUV (12bits) to BGR (24bits)
        video_pipe (object): GStreamer top-level pipeline
        video_sink (object): Gstreamer sink element
        video_sink_conf (string): Sink configuration
        video_source (string): Udp source ip and port
    """


   



    def __init__(self, port=5600):
        """Summary

        Args:
            port (int, optional): UDP port
        """

        Gst.init(None)

        self.port = port
        self._frame = None

        # [Software component diagram](https://www.ardusub.com/software/components.html)
        # UDP video stream (:5600)
        self.video_source = 'udpsrc port={}'.format(self.port)
        # [Rasp raw image](http://picamera.readthedocs.io/en/release-0.7/recipes2.html#raw-image-capture-yuv-format)
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) Iv)
        self.video_codec = '! application/x-rtp, payload=96 ! rtph264depay ! h264parse ! avdec_h264'
        # Python don't have nibble, convert YUV nibbles (4-4-4) to OpenCV standard BGR bytes (8-8-8)
        self.video_decode = \
            '! decodebin ! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert'
        # Create a sink to get data
        self.video_sink_conf = \
            '! appsink emit-signals=true sync=false max-buffers=2 drop=true'

        self.video_pipe = None
        self.video_sink = None

        self.run()

    def start_gst(self, config=None):
        """ Start gstreamer pipeline and sink
        Pipeline description list e.g:
            [
                'videotestsrc ! decodebin', \
                '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                '! appsink'
            ]

        Args:
            config (list, optional): Gstreamer pileline description list
        """

        if not config:
            config = \
                [
                    'videotestsrc ! decodebin',
                    '! videoconvert ! video/x-raw,format=(string)BGR ! videoconvert',
                    '! appsink'
                ]

        command = ' '.join(config)
        self.video_pipe = Gst.parse_launch(command)
        self.video_pipe.set_state(Gst.State.PLAYING)
        self.video_sink = self.video_pipe.get_by_name('appsink0')

    @staticmethod
    def gst_to_opencv(sample):
        """Transform byte array into np array

        Args:
            sample (TYPE): Description

        Returns:
            TYPE: Description
        """
        buf = sample.get_buffer()
        caps = sample.get_caps()
        array = np.ndarray(
            (
                caps.get_structure(0).get_value('height'),
                caps.get_structure(0).get_value('width'),
                3
            ),
            buffer=buf.extract_dup(0, buf.get_size()), dtype=np.uint8)
        return array
    
    
    def frame(self):
        """ Get Frame

        Returns:
            iterable: bool and image frame, cap.read() output
        """
        return self._frame

    def frame_available(self):
        """Check if frame is available

        Returns:
            bool: true if frame is available
        """
        return type(self._frame) != type(None)

    def run(self):
        """ Get frame to update _frame
        """

        self.start_gst(
            [
                self.video_source,
                self.video_codec,
                self.video_decode,
                self.video_sink_conf
            ])
    
        self.video_sink.connect('new-sample', self.callback)

    def callback(self, sink):
        sample = sink.emit('pull-sample')
        new_frame = self.gst_to_opencv(sample)
        self._frame = new_frame

        return Gst.FlowReturn.OK
    
if __name__ == '__main__':
    # Create the video object
    # Add port= if is necessary to use a different one
    rate = rospy.Rate(10)
    video = Video()
    ang =0
# Subscribe to the topic

   

    while True:
        # Wait for the next frame
        if not video.frame_available():
            continue
        img2 = video.frame()
        
        # cv2.imshow('frame', img2)
        img2 = cv2.resize(img2,(480,480))
        org_img = img2
        clahe_model = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        colorimage_b = clahe_model.apply(img2[:,:,0])
        colorimage_g = clahe_model.apply(img2[:,:,1])
        colorimage_r = clahe_model.apply(img2[:,:,2])
        colorimage_clahe = np.stack((colorimage_b,colorimage_g,colorimage_r), axis=2)
        img2 = colorimage_clahe
        img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV )
        cv2.medianBlur(img2 , 5)
         
        tl = (480,300)
        bl = (20 ,300)
        tr = (480,480)
        br = (20,480)

        # cv2.circle(img2, tl, 5, (0,0,255), -1)
        # cv2.circle(img2, bl, 5, (0,0,255), -1)
        # cv2.circle(img2, tr, 5, (0,0,255), -1)
        # cv2.circle(img2, br, 5, (0,0,255), -1)

        # cv2.imshow("data",img2)
             
        pts1 = np.float32([tl, bl, tr, br]) 
        pts2 = np.float32([[0, 0], [0, 480], [480, 0], [480, 480]]) 
        

       

        matrix = cv2.getPerspectiveTransform(pts1, pts2) 
        frame = img2
        frame = cv2.warpPerspective(img2, matrix, (480,480))
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

        


        # cv2.imshow("transform",frame)  
        l_h = cv2.getTrackbarPos("L - H", "Trackbars")
        l_s = cv2.getTrackbarPos("L - S", "Trackbars")
        l_v = cv2.getTrackbarPos("L - V", "Trackbars")
        u_h = cv2.getTrackbarPos("U - H", "Trackbars")
        u_s = cv2.getTrackbarPos("U - S", "Trackbars")
        u_v = cv2.getTrackbarPos("U - V", "Trackbars")
        
        lower = np.array([l_h,l_s,l_v])
        upper = np.array([u_h,u_s,u_v])
        
        frame_test = frame
        kernel = np.ones((5,5),np.uint8)

        mask = cv2.inRange(frame,lower,upper )
        dilation = cv2.dilate(mask,kernel,iterations = 2)
        erosion = cv2.erode(dilation,kernel,iterations = 1)
        dilation = cv2.dilate(erosion,kernel,iterations = 2)
        erosion= cv2.erode(dilation,kernel,iterations = 1)
      
        mask = erosion.copy()
         
        # cv2.imshow("maskimage",erosion) 
        histogram = np.sum(mask[mask.shape[2:478]], axis=0)
        midpoint = int(histogram.shape[0]/2)
        base = np.argmax(histogram)
 
        y = 480
        x = []
        yi = []

     

        # cv2.drawContours(frame, contours, -1 ,(0,255,0),-1)

        
        msk = mask.copy()
        pts = np.array([])
        while y>0:
             img = msk[y-40:y, base-100:base+100]
             contours, hierarchy = cv2.findContours(img,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
             for contour in contours:
                M = cv2.moments(contour)    
                if M["m00"] != 0:
                    cx = int(M["m10"]/M["m00"])
                    x.append(base)
                    base = base-50 + cx
            #  msk =  cv2.rectangle(msk, (base-100,y), (base+100,y-40), (255,255,255), 2)
            #  cv2.circle(frame_test, (base,y), 5, (0,0,255), -1)
             yi.append(y)
             y = y- 40          
       
        points = np.array(list(zip(x[:] , yi[:] )), np.int32).reshape(-1, 1, 2)

        isClosed = False
 
        # Green color in BGR
        color = (0, 255, 0)
        
        # Line thickness of 8 px
        thickness = 2
        
        # Using cv2.polylines() method
        # Draw a Green polygon with
        # thickness of 1 px
        # image = cv2.polylines(frame_test, [points],
        #                     isClosed, color,
        #                     thickness)
       
       
        if x:
            cv2.circle(msk, (x[0],420), 5, (0,255,0), -1)
            cv2.circle(msk, (240,460), 5, (0,255,0), -1)
             
 
            val = ang
            rospy.Subscriber('bluerov_heavy0/ref/yaw', Float64, getyaw)
            # rospy.Subscriber('/bluerov_heavy0/nav/filter/state', NavigationStatus, getangle, (ang)) #Controller1
            rospy.Subscriber('/bluerov_heavy0/nav/filter/state', NavigationStatus, getangle2centre, (x)) #Controller1
            # pub_sway = rospy.Publisher('/bluerov_heavy0/ref/sway', Float64)
         
        cv2.imshow("org",msk)
        # cv2.imshow("org2",org_img)    

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    rospy.spin()
