import cv2
import time

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

import pyzbar.pyzbar as pyzbar

# OUT_TOPIC = "/c270/image_raw"
OUT_TOPIC = "/qr"

PIPELINE = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw, format=BGR, width=640, height=480, framerate=30/1 ! nvimagesink"
# PIPELINE = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_200901010001-video-index0 ! videoconvert ! appsink"


cap = cv2.VideoCapture(PIPELINE, cv2.CAP_GSTREAMER)
qcd = cv2.QRCodeDetector()

time.sleep(1)

if not cap.isOpened():
    print("Can't open camera.")
    quit()
else:
    print("Opened camera. Press 'Q' to quit")

rospy.init_node("qr", anonymous=True)
br = CvBridge()
publisher = rospy.Publisher(OUT_TOPIC, Image, queue_size=2)
# header = Header()
# header.frame_id = "???s"


while True:

    ret, frame = cap.read()

    if not ret:
        print("Can't receive frame. ('q' to quit))")
        # break

    ret_qr, decoded_info, points, _ = qcd.detectAndDecodeMulti(frame)
    if ret_qr:
         for s, p in zip(decoded_info, points):
             if s:
                 print(s)
                 color = (0, 255, 0)
             else:
                 color = (0, 0, 255)
             frame = cv2.polylines(frame, [p.astype(int)], True, color, 8)
        #cv2.imshow(window_name, frame)


    # decodedObjects = pyzbar.decode(frame)
    # links = list(map(lambda decodedObject: decodedObject.data.decode("utf-8"), decodedObjects))

    

    # if len(links) > 0:
    #     print(f"QR Code(s): {links}")

    # cv2.imshow("Camera Feed", frame)
    if not rospy.is_shutdown():
        # header.stamp = rospy.Time.now()
        publisher.publish(br.cv2_to_imgmsg(frame))

    if cv2.waitKey(1) == ord("q"):
        break


cap.release()
cv2.destroyAllWindows()
