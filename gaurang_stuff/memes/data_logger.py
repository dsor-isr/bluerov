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
def getsub(msg):
        datasub = msg.data
        if(ang <= 180 and ang>= 135):
              val = 180 - ang  
              val = val * -1
              valn = val
        elif(ang >= 90 and ang <= 135):
                val = ang -90
                valn = val
        else:
                val = ang
                val = val * -1
        if(datasub == 1):
        #Put text over the plotted contour for the angle 
            cv2.putText(
                        org_img,
                        f"rope {round(val)}",
                        org,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        2,
                        cv2.LINE_AA,
                    )
        return datasub
# print(init)
rospy.Subscriber('/bluerov_heavy0/is_submerged',Bool, getsub)
#Check if rov is submerged or not 
rospy.init_node('nodesubmerged','listner','orient')
 


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
    
def getangle(msg, val):

            angl = msg.orientation.z    
            # print(angl)
            if(ang <= 180 and ang>= 135):
              val = 180 - ang  
              val = val * -1
            elif(ang >= 90 and ang <= 135):
                val = ang -90
            else:
                val = ang
                val = val * -1
            



            pub = rospy.Publisher('/bluerov_heavy0/ref/yaw', Float64)
            #Add val calculated as correction to the current heading 
            pub.publish(angl + val )
            print("val",val,"angle", angl, "target", angl+ val)
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
        # Cam -> CSI-2 -> H264 Raw (YUV 4-4-4 (12bits) I420)
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
    video = Video()
    ang =0
# Subscribe to the topic

   

    while True:
        # Wait for the next frame
        if not video.frame_available():
            continue
        time.sleep(0.25)
        img2 = video.frame()
        
        # cv2.imshow('frame', img2)
        img2 = cv2.resize(img2,(224,224))
        org_img = img2


        #CLAHE Equalization and stacking for color and feature enhancment

        small_img = img2
        clahe_model = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        colorimage_b = clahe_model.apply(img2[:,:,0])
        colorimage_g = clahe_model.apply(img2[:,:,1])
        colorimage_r = clahe_model.apply(img2[:,:,2])
        colorimage_clahe = np.stack((colorimage_b,colorimage_g,colorimage_r), axis=2)
        img2 = colorimage_clahe
        img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV )
        # cv2.imshow("image",img2)
        cv2.medianBlur(img2 , 5)
        frame = img2
        # cv2.imshow("mat",img2)

        #Image transform values for prespective transform
        #Orignal frame size 128 * 128 px
        tl = (175,100)
        bl = (50 ,100)
        tr = (200,200)
        br = (20,200)

        cv2.circle(img2, tl, 5, (0,0,255), -1)
        cv2.circle(img2, bl, 5, (0,0,255), -1)
        cv2.circle(img2, tr, 5, (0,0,255), -1)
        cv2.circle(img2, br, 5, (0,0,255), -1)


        pts1 = np.float32([tl, bl, tr, br]) 
        pts2 = np.float32([[0, 0], [0, 224], [224, 0], [224, 224]]) 
        

        matrix = cv2.getPerspectiveTransform(pts1, pts2) 
        frame = img2
        frame = cv2.warpPerspective(img2, matrix, (224,224))
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    

        # cv2.imshow("transform",frame)  
        #Wrapped true segmentation frame 








        frame_test = frame
        mask = cv2.inRange(frame,(5, 50, 40), (35, 200, 255) )
        kernel = np.ones((5,5),np.uint8)
        # cv2.imshow("maskimage",mask) 
        #True mask image segmented without erosion
        dilation = cv2.dilate(mask,kernel,iterations = 2)
        erosion = cv2.erode(dilation,kernel,iterations = 1)
        dilation = cv2.dilate(erosion,kernel,iterations = 2)
        erosion= cv2.erode(dilation,kernel,iterations = 1)
        # cv2.imshow("erosion",erosion) 
        #Eroded image without angle prediction
        img_np = np.array(erosion)
        contours, hierarchy = cv2.findContours(img_np,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
         #Retrive contours data from the eroded image
        if contours:
          c = max(contours, key=cv2.contourArea) 
            #Only retain contour with max area -  assuming that the rope is only yellow object in frame
          cnt = contours[0]
        area = cv2.contourArea(c)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        ctr = np.array(box).reshape((-1,1,2)).astype(np.int32)
        cv2.drawContours(org_img, [c], -1, (0, 255, 0), -1)
        #   (frame_test)
         
        #Box edge values for Contour ploting
        box = np.int0(box)
        x1 = box[0][0]
        y1 = box[0][1]
        x2 = box[1][0]
        y2 = box[1][1]  
        x3 = box[2][0]
        y3 = box[2][1]
        x4 = box[3][0]
        y4 = box[3][1]
        isClosed = True
        color = (255,0,0)
        thickness = 5
        corners = np.array([[x1, y1], [x2 ,y2] ,[x3, y3], [x4, y4]])
        image = cv2.polylines(org_img, [corners],
                        isClosed, color, thickness)
        angle = math.atan2((y1 -y2),(x1-x2))
        angle2 = math.atan2((y4 -y3),(x4-x3))
        #Both side angle calculation ,averaging of angle calc
        dist = math.dist([x1,y1],[x2,y2])
        dist1 = math.dist([x3,y3],[x4,y4])
        area = dist1 
        #Distance estimation for the width to depth conversion
         

        # print("area",area)
     
        
        # getarea(area)




        if(angle2!=180.0):    #Test without condition for the fake values

            # print(np.rad2deg((angle2)))
            # print(np.rad2deg(angle2))
            np.rad2deg((angle2))
            np.rad2deg(angle2)
        org = np.array([100,100])
        ang = np.rad2deg((angle2))
        valn = 0
        
        # print("ang pred", ang)
        # print("angle correction" ,val)
        # val = ang
        rospy.Subscriber('/bluerov_heavy0/is_submerged',Bool, getsub)
        rospy.Subscriber('bluerov_heavy0/ref/yaw', Float64, getyaw)
        rospy.Subscriber('/bluerov_heavy0/measurement/position', Measurement, getalt)
        rospy.Subscriber('/bluerov_heavy0/nav/filter/state', NavigationStatus, getangle, (valn))
        #getangle topic also publishing to the yaw angle
        #current issues  0 dead band  in the yaw angle in between the angle sub in function and yaw angle 


        ## ARUCO Marker detection algorithm
        gray_frame = cv2.cvtColor(org_img, cv2.COLOR_BGR2GRAY)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
        parameters =  cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(org_img)

        if markerCorners:
            for ids, corners in zip(markerIds, markerCorners):
                cv2.polylines(
                    org_img, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                )  #Plot poly lines for Marker corners detected
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()
                cv2.putText(   #Overlaw text for aruco calc value
                    org_img,
                    f"id: {ids[0]}",
                    top_right,  
                    cv2.FONT_HERSHEY_PLAIN,
                    1,
                    (200, 100, 0),
                    2,
                    cv2.LINE_AA,
                )   
        cv2.imshow("frame", org_img)
        #Display orignal image with overlap 
 
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    rospy.spin()
