HOST=192.168.42.100

RTPBIN_PARAMS="drop-on-latency=true"
DEVICE='/dev/video0'
CAPS='video/x-raw,format=I420,width=640,height=480,framerate=30/1'

gst-launch-1.0 -v rtpbin name=rtpbin $RTPBIN_PARAMS                                  \
        v4l2src device=$DEVICE ! $CAPS ! jpegenc ! rtpjpegpay ! rtpbin.send_rtp_sink_0 \
                  rtpbin.send_rtp_src_0 ! udpsink port=6000 host=$HOST                           \
                  rtpbin.send_rtcp_src_0 ! udpsink port=6001 host=$HOST sync=false async=false    \
                  udpsrc port=6005 host=$HOST ! rtpbin.recv_rtcp_sink_0
