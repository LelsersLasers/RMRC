import cv2
import numpy as np
from mss import mss
from PIL import Image

bb = {'top': 20, 'left': 20, 'width': 320, 'height': 420}

def screenshot():
       
    frame = None
    with mss() as sct:
        while True:
            sct_img = sct.grab(bb)
            np_image = np.array(sct_img)
            cv2.imshow('screencapture', np_image)

            if (cv2.waitKey(1) & 0xFF) == ord('g'):
                frame = np_image
                break

    print("All done gathering images.")

    output_path = "screencapture.jpg"
    cv2.imwrite(output_path, frame)
    print(f"Saved the final image as {output_path}")
    
    cv2.destroyAllWindows()

    return frame

