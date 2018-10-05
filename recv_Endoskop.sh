#!/bin/sh

DEST=192.168.42.220

RTPBIN_PARAMS="buffer-mode=0 do-retransmition=false drop-on-latency=true latency=250"
VIDEO_CAPS="application/x-rtp,media=(string)video,clock-rate=(int)90000,encoding-name=(string)JPEG"
VIDEO_DEC="rtpjpegdepay ! jpegdec ! videorate"
VIDEO_SINK="autovideosink sync=false"

gst-launch-1.0 \
    rtpbin name=rtpbin $RTPBIN_PARAMS \
        udpsrc caps=$VIDEO_CAPS port=6000 ! rtpbin.recv_rtp_sink_0 \
        rtpbin. ! $VIDEO_DEC ! $VIDEO_SINK \
        udpsrc port=6001 ! rtpbin.recv_rtcp_sink_0 \
        rtpbin.send_rtcp_src_0 ! udpsink port=6005 host=$DEST sync=false async=false
