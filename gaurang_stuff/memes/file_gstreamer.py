 #!/usr/bin/env python

import cv2
import gi
import numpy as np


import matplotlib.pyplot as plt
import cv2
from PIL import Image
import time
from skimage import data
from skimage import io,filters,feature

import math
from skimage.filters import roberts,sobel,scharr , prewitt,farid
from skimage.feature import canny
from skimage.feature import corner_harris
from skimage.feature import corner_peaks  



gi.require_version('Gst', '1.0')
from gi.repository import Gst


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

    while True:
        # Wait for the next frame
        if not video.frame_available():
            continue
        time.sleep(0.1)
        img2 = video.frame()
        
        # cv2.imshow('frame', img2)
        org_img = img2

        clahe_model = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        colorimage_b = clahe_model.apply(img2[:,:,0])
        colorimage_g = clahe_model.apply(img2[:,:,1])
        colorimage_r = clahe_model.apply(img2[:,:,2])
        colorimage_clahe = np.stack((colorimage_b,colorimage_g,colorimage_r), axis=2)
        img2 = colorimage_clahe
        img2 = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV )

        cv2.medianBlur(img2 , 5)
        frame = img2
        frame_test = frame
        mask = cv2.inRange(frame,(5, 50, 40), (35, 200, 255) )
        kernel = np.ones((5,5),np.uint8)
        # cv2_imshow(mask) 
        dilation = cv2.dilate(mask,kernel,iterations = 1)
        erosion = cv2.erode(dilation,kernel,iterations = 2)
        dilation = cv2.dilate(erosion,kernel,iterations = 4)
        erosion= cv2.erode(dilation,kernel,iterations = 3)
        # # cv2_imshow(erosion) 
        img_np = np.array(erosion)
        contours, hierarchy = cv2.findContours(img_np,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if contours:
          c = max(contours, key=cv2.contourArea)
          cnt = contours[0]
        area = cv2.contourArea(c)
        rect = cv2.minAreaRect(c)
        box = cv2.boxPoints(rect)
        ctr = np.array(box).reshape((-1,1,2)).astype(np.int32)
        cv2.drawContours(org_img, [c], -1, (0, 255, 0), -1)
        # cv2_imshow(frame_test)
        
        

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
        angle2 = math.atan2((y3 -y4),(x3-x4))
        if(angle2!=180.0):

            print(np.rad2deg((angle2)))
            # print(np.rad2deg(angle2))
        org = np.array([100,100])
        ang = np.rad2deg((angle2))
        
        cv2.putText(
                    org_img,
                    f"rope {round(ang)}",
                    org,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.3,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

        gray_frame = cv2.cvtColor(org_img, cv2.COLOR_BGR2GRAY)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
        parameters =  cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)

        
        markerCorners, markerIds, rejectedCandidates = detector.detectMarkers(org_img)

        if markerCorners:
            for ids, corners in zip(markerIds, markerCorners):
                cv2.polylines(
                    org_img, [corners.astype(np.int32)], True, (0, 255, 255), 4, cv2.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()
                cv2.putText(
                    org_img,
                    f"id: {ids[0]}",
                    top_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    4,
                    (200, 100, 0),
                    2,
                    cv2.LINE_AA,
                )   
        cv2.imshow("frame", org_img)




        if cv2.waitKey(1) & 0xFF == ord('q'):
            break