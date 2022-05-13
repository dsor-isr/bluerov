#include <ros/ros.h>
#include <medusa_gimmicks_library/MedusaGimmicks.h>

class CameraServerNode {

    public:

        /**
         * @brief Construct a new Server Relay Node object
         * 
         * @param nh 
         * @param nh_p 
         */
        CameraServerNode(ros::NodeHandle * nh, ros::NodeHandle * nh_p);
        
        /**
         * @brief Destroy the Server Relay Node object
         * 
         */
        ~CameraServerNode();


    private:

        /**
         * @brief ROS node handlers
         */
        ros::NodeHandle nh_, nh_p_;

        /**
         * @brief Method to initialize parameters from the ROS parameter server
         */
        void initializeParameters();

        /**
         * @brief 
         */
        void startStream(); 

        /* Select which mapping to use, i.e. UDP -> HTTP or HTTP -> HTTP */
        std::string stream_type_{"UDP"};
        unsigned int stream_port_{5600};
        std::string stream_addresss_{"192.168.2.1"};

        /* UDP Stream configurations (WHEN RECEIVING FROM UDP) */
        const std::string udp_select{"udpsrc"};        
        const std::string udp_config{"caps=application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)H264 ! rtph264depay ! avdec_h264 ! videoconvert ! appsink emit-signals=true sync=false max-buffers=2 drop=true"};

        /* HTTP (BROADCASTER configuration - send the data through a new HTTP stream) */
        unsigned int broadcast_http_port_{8080};
        std::string broadcast_http_address_{"/"};
        unsigned int broadcast_number_threads_{2};

        /* Image quality*/
        int image_quality_{30};
};