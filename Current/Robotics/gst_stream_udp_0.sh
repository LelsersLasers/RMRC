gst-launch-1.0 nvarguscamerasrc sensor-id=0 sensor-mode=4 \
        ! 'video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=60/1' \
    ! nvvidconv flip-method=0 \
        ! 'video/x-raw(memory:NVMM),format=RGBA,width=640,height=360' \
    ! comp. nvarguscamerasrc sensor-id=1 sensor-mode=4 \
        ! 'video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=60/1' \
    ! nvvidconv flip-method=0 \
        ! 'video/x-raw(memory:NVMM),format=RGBA,width=640,height=360' \
    ! nvcompositor name=comp sink_0::xpos=0 sink_0::ypos=0 sink_0::width=640 sink_0::height=360 sink_1::xpos=640 sink_1::ypos=0 sink_1::width=640 sink_1::height=360 \
    ! nvvidconv \
        ! 'video/x-raw(memory:NVMM),format=NV12,framerate=30/1' \
    ! nvjpegenc \
    ! rtpjpegpay \
    ! udpsink host=192.168.1.2 port=5010
