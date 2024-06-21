import cv2
import numpy as np
import pyzbar.pyzbar as pyzbar


def qr_detect_and_draw(input_frame, ouput_frame):
    decoded_objects = pyzbar.decode(input_frame)

    links = []

    fontScale = 0.25
    fontColor = (0, 0, 255)
    thickness = 1
    lineType = 2

    for decoded_object in decoded_objects:
        link = decoded_object.data.decode("utf-8")
        links.append(link)

        points = decoded_object.polygon
        cv2.polylines(ouput_frame, [np.array(points, np.int32)], True, (0, 255, 0), 2)

        x, y, _, _ = decoded_object.rect
        cv2.putText(
            ouput_frame,
            link,
            (x, y),
            cv2.FONT_HERSHEY_SIMPLEX,
            fontScale,
            fontColor,
            thickness,
            lineType,
        )

    return links