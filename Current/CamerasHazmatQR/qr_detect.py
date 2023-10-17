import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar


def qr_detect(frame):
    decoded_objects = pyzbar.decode(frame)

    links = []

    for decoded_object in decoded_objects:
        link = decoded_object.data.decode("utf-8")
        links.append(link)

        points = decoded_object.polygon
        cv2.polylines(frame, [np.array(points, np.int32)], True, (0, 255, 0), 3)

        x, y, _, _ = decoded_object.rect
        cv2.putText(frame, link, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

    return links
