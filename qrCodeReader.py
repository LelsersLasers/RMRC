from __future__ import annotations

import cv2
import time

import pyzbar.pyzbar as pyzbar


cap = cv2.VideoCapture(0)


if not cap.isOpened():
    print("Can't open camera.")
    quit()
else:
    print("Opened camera.")


time.sleep(1)


while True:

    ret, frame = cap.read()

    if not ret:
        print("Can't receive frame. ('q' to quit))")
        # break


    decodedObjects = pyzbar.decode(frame)
    links = list(map(lambda decodedObject: decodedObject.data.decode("utf-8"), decodedObjects))

    if len(links) > 0:
        print(f"QR Code(s): {links}")

    cv2.imshow("Camera Feed", frame)

    if cv2.waitKey(1) == ord("q"):
        break


cap.release()
cv2.destroyAllWindows()
