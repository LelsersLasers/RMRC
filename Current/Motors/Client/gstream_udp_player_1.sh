gst-launch-1.0 udpsrc port=5010 ! application/x-rtp,encoding-name=JEPD ! rtpjpegdepay ! jpegdec ! autovideosink
