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

x =0 
y =0 


 
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

# Subscribe to the topic

   

    while True:
        # Wait for the next frame
        if not video.frame_available():
            continue
        # time.sleep(0.25)
        img2 = video.frame()
        
        cv2.imshow('frame', img2)
        img2 = cv2.resize(img2,(480,480))
        org_img = img2


        #CLAHE Equalization and stacking for color and feature enhancment

     
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

        #Image transform values for prespective transform
        #Orignal frame size 128 * 128 px
        tl = (80,250)
        bl = (400 ,250)
        tr = (480,480)
        br = (0,480)

        cv2.circle(img2, tl, 5, (0,0,255), -1)
        cv2.circle(img2, bl, 5, (0,0,255), -1)
        cv2.circle(img2, tr, 5, (0,0,255), -1)
        cv2.circle(img2, br, 5, (0,0,255), -1)

        pts1 = np.float32([tl, bl, tr, br]) 
        pts2 = np.float32([[0, 0], [0, 480], [480, 0], [480, 480]]) 
        
        cv2.imshow("mat",img2)

        matrix = cv2.getPerspectiveTransform(pts1, pts2) 
        frame = img2
        frame = cv2.warpPerspective(img2, matrix, (480,480))
        # frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    

        cv2.imshow("transform",frame)  
        #Wrapped true segmentation frame 








        frame_test = frame
        mask = cv2.inRange(frame,(5, 50, 80), (35, 200, 255) )
        kernel = np.ones((5,5),np.uint8)
       
        img_np = np.array(mask)
        contours, hierarchy = cv2.findContours(img_np,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
         #Retrive contours data from the eroded image
        c = 0
        # if contours:
        #   c = max(contours, key=cv2.contourArea) 
        #     #Only retain contour with max area -  assuming that the rope is only yellow object in frame
        #   cnt = contours[0]
        # area = cv2.contourArea(c)
        # rect = cv2.minAreaRect(c)
        # box = cv2.boxPoints(rect)
        # ctr = np.array(box).reshape((-1,1,2)).astype(np.int32)
        # cv2.drawContours(org_img, [c], -1, (0, 255, 0), -1)
        # #   (frame_test)
         
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    rospy.spin()
480