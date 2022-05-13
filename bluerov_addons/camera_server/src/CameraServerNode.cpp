#include "CameraServerNode.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <nadjieb/mjpeg_streamer.hpp>

/* Class constructor for server relay node*/
CameraServerNode::CameraServerNode(ros::NodeHandle * nh, ros::NodeHandle * nh_p) : nh_(*nh), nh_p_(*nh_p) {

    /* Initialize all the parameters */
    this->initializeParameters();

    /* Consume the UDP or HTTP stream and broadcast it to the network through HTTP */
    this->startStream();
}

/* Class destructor for server relay node */
CameraServerNode::~CameraServerNode() {}

/* Get Parameters from the parameter server */
void CameraServerNode::initializeParameters() {

    /* Define the parameters that define where to get the video feed from (and if it comes from UDP or already in HTTP) */
    this->stream_type_ = MedusaGimmicks::getParameters<std::string>(this->nh_p_, "stream_type", "UDP");
    this->stream_port_ = MedusaGimmicks::getParameters<int>(this->nh_p_, "stream_port", 5600);
    this->stream_addresss_ = MedusaGimmicks::getParameters<std::string>(this->nh_p_, "stream_address", "192.168.2.1");

    /* Define the HTTP broadcasting parameters and number of threads */
    this->broadcast_http_port_ = MedusaGimmicks::getParameters<int>(this->nh_p_, "broadcast_port", 8080);
    this->broadcast_http_address_ = MedusaGimmicks::getParameters<std::string>(this->nh_p_, "broadcast_address", "/");
    this->broadcast_number_threads_ = MedusaGimmicks::getParameters<int>(this->nh_p_, "number_threads", 2);

    /* Define the level of compression to apply to the image */
    this->image_quality_ = MedusaGimmicks::getParameters<int>(this->nh_p_, "image_quality", 30);
}

/* Method that will be running the server until node shutdown */
void CameraServerNode::startStream() {

    using MJPEGStreamer = nadjieb::MJPEGStreamer;

    cv::VideoCapture cap;

    /* Subscribe to a UDP or HTTP video stream */
    if (std::string("UDP") == this->stream_type_) {
        cap = cv::VideoCapture("udpsrc" + 
            std::string(" address=") + this->stream_addresss_ + 
            std::string(" port=") + std::to_string(this->stream_port_) +
            std::string(" caps=application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink emit-signals=true sync=false max-buffers=2 drop=true"),
            cv::CAP_GSTREAMER);
    } else if (std::string("HTTP") == this->stream_type_) {
        cap = cv::VideoCapture("http://" + this->stream_addresss_);
    } else {
        ROS_WARN_STREAM("Could not connect to the type of stream requested. Only support UDP or HTTP");
        return;
    }

    if (!cap.isOpened()) {
        ROS_WARN_STREAM("VideoCapture not opened");
        exit(EXIT_FAILURE);
    }

    // Vector of qualities
    std::vector<int> qualities = {25, 50, 75, 95};

    // Select the quality to which to broadcast the video frames
    std::vector<std::vector<int>> params;
    
    for(unsigned int i = 0; i < qualities.size(); i++) {
        params.push_back({cv::IMWRITE_JPEG_QUALITY, qualities[i]});
    }

    MJPEGStreamer streamer;

    // By default "/shutdown" is the target to graceful shutdown the streamer
    // if you want to change the target to graceful shutdown:
    streamer.setShutdownTarget("/stop");

    // Start the stream with N default workers
    streamer.start(this->broadcast_http_port_, this->broadcast_number_threads_);

    // Visit /shutdown or another defined target to stop the loop and graceful shutdown
    while (streamer.isRunning()) {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "frame not grabbed\n";
            exit(EXIT_FAILURE);
        }

        std::vector<uchar> buff_bgr;

        // Iterate over the 10 video qualities and generate 10 levels of compression
        for(unsigned int i = 0; i < qualities.size(); i++) {

            if(i == 0) {
                cv::Mat frame_small;
                cv::resize(frame, frame_small, cv::Size(frame.cols * 0.5, frame.rows * 0.5), 0, 0, cv::INTER_LANCZOS4);

                cv::imencode(".jpg", frame_small, buff_bgr, params[i]);

                // Convert the image data to a string
                std::string image_data = std::string(buff_bgr.begin(), buff_bgr.end());

                // Publish the stream 
                streamer.publish(this->broadcast_http_address_ + std::to_string(qualities[i]), image_data);
            } else {

                // Encode the image
                cv::imencode(".jpg", frame, buff_bgr, params[i]);

                // Convert the image data to a string
                std::string image_data = std::string(buff_bgr.begin(), buff_bgr.end());

                // Publish the stream 
                streamer.publish(this->broadcast_http_address_ + std::to_string(qualities[i]), image_data);
            }
        }

        // http://localhost:8080/bgr
        //std::vector<uchar> buff_bgr;
        //cv::imencode(".jpg", frame, buff_bgr, params);

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    streamer.stop();
}

int main(int argc, char ** argv) {

    ros::init(argc, argv, "camera_server_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_p("~");

    ROS_INFO("main: instantiating an object of type CameraServerNode");

    /* Instantiate the ServerRelay Node */
    CameraServerNode CameraServerNode(&nh, &nh_p);

    return 0;
}