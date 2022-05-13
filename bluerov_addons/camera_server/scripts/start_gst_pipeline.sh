#!/bin/bash

UDP_PORT_BLUEROV=$1
UDP_PORT_LENOVO_TO_NETWORK=$2
UDP_PORT_LENOVO_TO_LOCAL=$(($UDP_PORT_LENOVO_TO_NETWORK+1))
UDP_PORT_LENOVO_TO_SAVE=$(($UDP_PORT_LENOVO_TO_NETWORK+2))
NETWORK_IP=$3
UDP_PORT_NETWORK=$4
X264_ENC_PRESET=$5
PAYLOAD_MTU_NETWORK=$6
VIDEO_FILE_DIR=$7

tmux start

if [[ $# -ne 7 ]]
  then  # RUNNING DEFAULT CONFIGURATIONS
    echo "Please specify the necessary arguments:"
    echo ""
    echo "      $0 [udp_port_bluerov] [udp_port_split] [network_upstream_ip] [udp_port_network] [x264_enc_preset] [payload_mtu] [video_file_dir]"
    echo ""
    echo "[udp_port_bluerov]          Bluerov UDP port to which video stream is being served."
    echo "[udp_port_split]            Starting UDP port to split locally."
    echo "[network_upstream_ip]       Upstream machine IP."
    echo "[udp_port_network]          Upstream UDP port to which video is being streamed."
    echo "[x264_enc_preset]           H264 enconding preset (None-0, ultrafast-1, superfast-2, veryfast-3, faster-4, fast-5, medium-6, slow-7, slower-8, veryslow-9, placebo-10)"
    echo "[payload_mtu]               Maximum transmission unit of each UDP packet (to be sent to network)."
    echo "[video_file_dir]            Directory of video file save location."
    echo
    echo "Starting script with default values..."
    echo
    echo "      $0 udp_port_bluerov=5600 udp_port_split=5605 network_upstream_ip=10.0.8.162 udp_port_network=5600 x264_enc_preset=superfast payload_mtu=1392 video_file_dir=/home/dsor/Videos/"

    # 1. Split the stream into 2 streams (default: port 5600 -> to ports 5605, 5606 and 5607)
    GST_MULTIUDPSINK_CMD="gst-launch-1.0 udpsrc port=5600 \
                        ! multiudpsink clients=192.168.2.1:5605,192.168.2.1:5606,192.168.2.1:5607"

    # 2. Decode the stream (at 192.168.2.1:5605) and then encode it more efficiently and send to the network (10.0.8.162:5600)
    GST_NETWORK_CMD="gst-launch-1.0 udpsrc port=5605 ! application/x-rtp,encoding-name=H264,payload=96,framerate=30/1 \
                        ! rtph264depay ! avdec_h264 ! videoconvert ! x264enc speed-preset=superfast tune=zerolatency \
                        ! rtph264pay mtu=1392 ! udpsink host=10.0.8.162 port=5600"
    
    # 3. Watch the stream on port 5606
    GST_DISPLAY_LOCAL_CMD="gst-launch-1.0 udpsrc port=5606 ! application/x-rtp,encoding-name=H264,payload=96 \
                        ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=10000 sync=false"
    
    # 4. Record the stream on port 5607
    FILE_NAME="bluerov-$(date +%d-%b-%Y-%H-%M)"
    GST_SAVE_VIDEO_CMD="gst-launch-1.0 -e udpsrc port=5607 ! application/x-rtp, clock-rate=90000, encoding-name=H264, payload=96 \
                        ! rtpjitterbuffer ! rtph264depay ! mpegtsmux ! filesink sync=false location=/home/dsor/Videos/${FILE_NAME}.ts"

    # new tmux session for the gstreamer pipeline
    tmux new-session -d -s gst_pipeline -n gst_multi_udpsink

    # create new pane for the multi udp sink stream
    tmux send-keys -t gst_pipeline:0 "${GST_MULTIUDPSINK_CMD}" Enter

    sleep 1

    # create new pane for the local stream display
    tmux new-window -t gst_pipeline:1 -n gst_local_display
    tmux send-keys -t gst_pipeline:1 "${GST_DISPLAY_LOCAL_CMD}" Enter

    sleep 1

    # create new pane for the network upstream 
    tmux new-window -t gst_pipeline:2 -n gst_network_upstream
    tmux send-keys -t gst_pipeline:2 "${GST_NETWORK_CMD}" Enter

    sleep 1

    # new tmux session for the gstream video save to file
    tmux new-session -d -s gst_save_file -n gst_save_file
    tmux send-keys -t gst_save_file:0 "${GST_SAVE_VIDEO_CMD}" Enter

    echo "The GStreamer pipeline is deployed."

else 
    if tmux ls | grep -q 'gst_pipeline'
    then
        echo "Found tmux session already running the GStreamer pipeline."
    else
        echo "Launching a new tmux session with the GStreamer pipeline..."

        # 1. Split the stream into 2 streams (default: port 5600 -> to ports 5605, 5606 and 5607)
        GST_MULTIUDPSINK_CMD="gst-launch-1.0 udpsrc port=${UDP_PORT_BLUEROV} \
                            ! multiudpsink clients=192.168.2.1:${UDP_PORT_LENOVO_TO_NETWORK},192.168.2.1:${UDP_PORT_LENOVO_TO_LOCAL},192.168.2.1:${UDP_PORT_LENOVO_TO_SAVE}"

        # 2. Decode the stream (at 192.168.2.1:5605) and then encode it more efficiently and send to the network (10.0.8.162:5600)
        GST_NETWORK_CMD="gst-launch-1.0 udpsrc port=${UDP_PORT_LENOVO_TO_NETWORK} ! application/x-rtp,encoding-name=H264,payload=96,framerate=30/1 \
                            ! rtph264depay ! avdec_h264 ! videoconvert ! x264enc speed-preset=${X264_ENC_PRESET} tune=zerolatency \
                            ! rtph264pay mtu=${PAYLOAD_MTU_NETWORK} ! udpsink host=${NETWORK_IP} port=${UDP_PORT_NETWORK}"
        
        # 3. Watch and record stream in port 5607 locally, to a x264 compatible container (.ts, etc)
        GST_DISPLAY_LOCAL_CMD="gst-launch-1.0 udpsrc port=${UDP_PORT_LENOVO_TO_LOCAL} ! application/x-rtp,encoding-name=H264,payload=96 \
                            ! rtph264depay ! avdec_h264 ! videoconvert ! autovideosink fps-update-interval=10000 sync=false"
        
        # 4. Decode the stream (at 192.168.2.1:5605) and then encode it more efficiently and send to the network (10.0.8.162:5600)

        FILE_NAME="bluerov-$(date +%d-%b-%Y-%H-%M)"
        GST_SAVE_VIDEO_CMD="gst-launch-1.0 -e udpsrc port=${UDP_PORT_LENOVO_TO_SAVE} ! application/x-rtp, clock-rate=90000, encoding-name=H264, payload=96 \
                            ! rtpjitterbuffer ! rtph264depay ! mpegtsmux ! filesink sync=false location=${VIDEO_FILE_DIR}${FILE_NAME}.ts"

        # new tmux session for the gstreamer pipeline
        tmux new-session -d -s gst_pipeline -n gst_multi_udpsink

        # create new pane for the multi udp sink stream
        tmux send-keys -t gst_pipeline:0 "${GST_MULTIUDPSINK_CMD}" Enter

        sleep 1

        # create new pane for the local stream display
        tmux new-window -t gst_pipeline:1 -n gst_local_display
        tmux send-keys -t gst_pipeline:1 "${GST_DISPLAY_LOCAL_CMD}" Enter

        sleep 1

        # create new pane for the network upstream 
        tmux new-window -t gst_pipeline:2 -n gst_network_upstream
        tmux send-keys -t gst_pipeline:2 "${GST_NETWORK_CMD}" Enter

        sleep 1

        # new tmux session for the gstream video save to file
        tmux new-session -d -s gst_save_file -n gst_save_file
        tmux send-keys -t gst_save_file:0 "${GST_SAVE_VIDEO_CMD}" Enter

        echo "The GStreamer pipeline is deployed."
    fi
fi
