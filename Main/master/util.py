import os
import cv2


# ---------------------------------------------------------------------------- #
def read_last_line(f):
    # with open('filename.txt', 'rb') as f:
    try:  # catch OSError in case of a one line file 
        f.seek(-2, os.SEEK_END)
        while f.read(1) != b'\n':
            f.seek(-2, os.SEEK_CUR)
    except OSError:
        f.seek(0)
        
    last_line = f.readline().decode()
    return last_line
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def draw_fps_text(frame, fps):
    font = cv2.FONT_HERSHEY_SIMPLEX
    bottomLeftCornerOfText = (5, frame.shape[0] - 5)
    fontScale = 0.5
    fontColor = (0, 255, 0)
    thickness = 1
    lineType = 2

    text = "FPS: %.0f" % fps
    cv2.putText(frame, text, bottomLeftCornerOfText, font, fontScale, fontColor, thickness, lineType)

def draw_ratio_bar(frame, ratio, active, loading = False):
    ratio = min(ratio, 1)
    w = ratio * (frame.shape[1] - 10)

    color = (0, 255, 0) if loading else ((0, 0, 255) if active else (255, 255, 0))

    cv2.line(frame, (5, 5), (5 + int(w), 5), color, 3)
# ---------------------------------------------------------------------------- #