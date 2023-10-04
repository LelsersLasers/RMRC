# cap_args = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink"

cap_args = {
    "webcam1": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink",
    "webcam2": "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_348E60A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink",
    "ir": "v4l2src device=/dev/v4l/by-id/usb-GroupGets_PureThermal__fw:v1.3.0__8003000b-5113-3238-3233-393800000000-video-index0 ! videoconvert ! appsink",
}


import cv2
import time
from levenshtein import *
from cleanUp import *
import numpy as np
import argparse
import mahotas
import pytesseract
from multiprocessing import Process, Queue
import pyzbar.pyzbar as pyzbar
import argparse
import json
import base64


QUIT_KEY = "q"
HAZMAT_TOGGLE_KEY = "h"
QR_TOGGLE_KEY = "r"
HAZMAT_CLEAR_KEY = "c"
QR_CLEAR_KEY = "x"

HAZMAT_MIN_DELAY = 0.1 # TODO?
CAMERA_WAKEUP_TIME = 0.5
HAZMAT_FRAME_SCALE = 1
HAZMAT_DELAY_BAR_SCALE = 10 # in seconds
QR_TIME_BAR_SCALE = 0.1 # in seconds
SERVER_FRAME_SCALE = 1

MAIN_FILE = "states/state.json"
SERVER_FILE = "states/server_state.json"

# What main thread sends
START_STATE_MAIN = {
    "frame": None,
    "run_hazmat": False,
    "quit": False,
    "clear_all_found": 0,
}

# What hazmat thread sends
START_STATE_HAZMAT = {
    "hazmat_delta": 1 / 10,
    "hazmat_frame": None,
    # "cleared_all_found": False,
}

MAIN_STATE = {
    "frame": "",
    "w": 1,
    "h": 1,
}
SERVER_STATE = {}

def write_state():
    # Write state to file
    with open(MAIN_FILE, "w") as f:
        json.dump(MAIN_STATE, f)

def read_state():
	global SERVER_STATE
	try:
		with open(SERVER_FILE, "r") as f:
			SERVER_STATE = json.load(f)
	except:
		time.sleep(0.1)
		read_state()


def processScreenshot(img, val):
    # CHANGE THRESHOLD AS NEEDED
    lowerThresh = np.array([0, 0, 0])  # lower thresh for black
    upperThresh = np.array([val, val, val])  # upper thresh for white

    # Create a binary mask based on the color threshold range
    mask = cv2.inRange(img, lowerThresh, upperThresh)

    # apply mask
    binary = np.zeros_like(img)
    binary[mask > 0] = 255

    img = cv2.bitwise_not(binary)

    # if args["debug"]:
    #     cv2.imshow("Inverted Binary Image", img)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(img, (5, 5), 0)
    # cv2.imshow("image", img)

    T = mahotas.thresholding.rc(blurred)
    thresh = img.copy()
    thresh[thresh > T] = 255
    thresh[thresh < 255] = 0
    thresh = cv2.bitwise_not(thresh)

    _, thresh = cv2.threshold(gray, 50, 255, 0)
    contours, _ = cv2.findContours(thresh, 1, 2)
    # print("Number of contours detected:", len(contours))

    imageList = []
    for i, cnt in enumerate(contours):
        x1, y1 = cnt[0][0]
        approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(cnt)
            ratio = float(w) / h
            if w > 63:
                count = i
                if ratio >= 0.8 and ratio <= 1.2:
                    img = cv2.drawContours(img, [cnt], -1, (255, 0, 0), 3)
                    cv2.putText(
                        img, "Square", (x1, y1), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
                    )

                    # if args["debug"]:
                    #     cv2.imshow("squares", img)

                    cropped = img[y : y + h, x : x + w]
                    cropped = cv2.bitwise_and(cropped, img[y : y + h, x : x + w])
                    rows, cols = cropped.shape[:2]
                    matrixCW45 = cv2.getRotationMatrix2D((cols / 2, rows / 2), -45, 1)
                    cw45 = cv2.warpAffine(cropped, matrixCW45, (cols, rows))

                    matrixCCW45 = cv2.getRotationMatrix2D((cols / 2, rows / 2), 45, 1)
                    ccw45 = cv2.warpAffine(cropped, matrixCCW45, (cols, rows))

                    matrixCCW135 = cv2.getRotationMatrix2D((cols / 2, rows / 2), 135, 1)
                    ccw135 = cv2.warpAffine(cropped, matrixCCW135, (cols, rows))

                    matrixCW135 = cv2.getRotationMatrix2D((cols / 2, rows / 2), -135, 1)
                    cw135 = cv2.warpAffine(cropped, matrixCW135, (cols, rows))

                    imageList.append((cw45, count, cnt))
                    imageList.append((ccw45, count, cnt))
                    imageList.append((ccw135, count, cnt))
                    imageList.append((cw135, count, cnt))
                    imageList.append((cropped, count, cnt))

    myDict = {}
    for i, (image, count, cnt) in enumerate(imageList):
        width, height, _ = image.shape

        # TODO: how do these calculations work???
        x1 = int(0)
        y1 = int(height / 2 - (height * 0.15))
        x2 = int(width)
        y2 = int(height / 2 + (height * 0.25))

        onlyText = image[y1:y2, x1:x2]
        # if args["debug"]:
        #     cv2.rectangle(image, (x1, y1), (x2, y2), (225, 0, 0), 2)
        #     cv2.imshow(f"image {i}", onlyText)
        text = pytesseract.pytesseract.image_to_string(onlyText, config="--psm 6")
        text = removeSpecialCharacter(text)
        if text != "":
            myDict.update({text: (cnt)})

    words = [
        "explosive",
        "blasting agent",
        "non flammable gas",
        "inhalation hazard",
        "infectious substance",
        "flammable liquid",
        "spontaneously combustible",
        "dangerous when wet",
        "oxidizer",
        "organic peroxide",
        "poison",
        "corrosive",
        "flammable gas",
    ]

    correct_tups = []
    for key in myDict:
        word = key
        closest, distance = checkList(word, words)
        ratio = distance / len(closest)
        if ratio <= 0.55:
            cnt = myDict[key]
            tup = (closest, cnt)
            correct_tups.append(tup)

    correct_tups = remove_dups(correct_tups, lambda x: x[0])
    return correct_tups


def remove_dups(list, comp):
    new_list = []
    for item in list:
        if comp(item) not in [comp(x) for x in new_list]:
            new_list.append(item)
    return new_list


def findMax(list):
    return max(list, key=len)


class Toggler:
    def __init__(self, start_state=False):
        self.state = start_state

    def toggle(self):
        self.state = not self.state

    def get(self):
        return self.state
    
    def __bool__(self):
        return self.state
    
    def __str__(self):
        return str(self.state)

def hazmat_main(main_queue, hazmat_queue):
    time.sleep(CAMERA_WAKEUP_TIME)

    t0 = time.time()
    t1 = time.time()
    delta = 1 / 10

    all_found = []
    frame = None

    state_main = START_STATE_MAIN
    state_hazmat = START_STATE_HAZMAT

    while not state_main["quit"]:

        clear_all_found = False
        try:
            state_main = main_queue.get_nowait()
            clear_all_found = clear_all_found or state_main["clear_all_found"] > 0
        except:
            pass

        if clear_all_found:
            all_found = []
            print("Cleared all found hazmat labels.")

        if state_main["frame"] is not None:
            frame = state_main["frame"]
            frame = cv2.resize(frame, (0, 0), fx=HAZMAT_FRAME_SCALE, fy=HAZMAT_FRAME_SCALE)

            if state_main["run_hazmat"]:

                threshVals = [90, 100, 110, 120, 130, 140, 150, 160, 170]
                found_this_frame = []
                for threshVal in threshVals:
                    received_tups = processScreenshot(frame, threshVal)

                    for r in received_tups:
                        found_this_frame.append(r)
                        all_found.append(r[0])

                found_this_frame = remove_dups(found_this_frame, lambda x: x[0])
                all_found = list(set(all_found))

                fontScale = 0.5
                fontColor = (0, 0, 255)
                thickness = 1
                lineType = 2

                for found in found_this_frame:
                    text, cnt = found

                    frame = cv2.drawContours(frame, [cnt], -1, (255, 0, 0), 3)
                    x, y, w, h = cv2.boundingRect(cnt)

                    cv2.rectangle(frame, (x, y), (x + w, y + h), (225, 0, 0), 4)

                    corner = (x + 5, y + 15)

                    cv2.putText(
                        frame,
                        text,
                        corner,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale,
                        fontColor,
                        thickness,
                        lineType,
                    )
                
                if len(found_this_frame) > 0:
                    print([x[0] for x in found_this_frame])
                    print(all_found)

            unscale = 1 / HAZMAT_FRAME_SCALE
            state_hazmat["hazmat_frame"] = cv2.resize(frame, (0, 0), fx=unscale, fy=unscale)

        t1 = time.time()
        delta = t1 - t0
        t0 = t1

        sleep_time = max(HAZMAT_MIN_DELAY - delta, 0)
        time.sleep(sleep_time)

        state_hazmat["hazmat_delta"] = delta

        hazmat_queue.put_nowait(state_hazmat)

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


def main(main_queue, hazmat_queue, debug, video_capture_zero):
    global SERVER_STATE, MAIN_STATE

    print("Starting camera...")

    caps = {}
    if video_capture_zero:
        caps["webcam1"] = cv2.VideoCapture(0)
    else:
        for key, value in cap_args.items():
            caps[key] = cv2.VideoCapture(value, cv2.CAP_GSTREAMER)

    for key, cap in caps.items():
        if not cap.isOpened():
            raise RuntimeError(
                f"Can't open camera {key}. Are the cap_args set right? Is the camera plugged in?"
            )


    time.sleep(CAMERA_WAKEUP_TIME)


    print(f"\nPress '{QUIT_KEY}' to quit.")
    print(f"Press '{HAZMAT_TOGGLE_KEY}' to toggle running hazmat detection.")
    print(f"Press '{HAZMAT_CLEAR_KEY}' to clear all found hazmat labels.")
    print(f"Press '{QR_TOGGLE_KEY}' to toggle running QR detection.")
    print(f"Press '{QR_CLEAR_KEY}' to clear all found QR codes.\n")

    t0 = time.time()
    t1 = time.time()
    delta = 1 / 30

    run_hazmat = Toggler(False)
    run_qr = Toggler(False)

    all_qr_found = []

    state_main = START_STATE_MAIN
    state_hazmat = START_STATE_HAZMAT

    last_hazmat_update = time.time()
    hazmat_frame_orginal = None

    while True:
        frames = {}
        for key, cap in caps.items():
            ret, frame = cap.read()

            if not ret or frame is None:
                print("Exiting ...")
                return caps

            frames[key] = frame

        frame = frames["webcam1"]

        webcam1_shape = frames["webcam1"].shape
        if video_capture_zero:
            ir_frame = frames["webcam1"]
        else:
            ir_frame = cv2.resize(frames["ir"], (webcam1_shape[1], webcam1_shape[0]))

        try:
            state_hazmat = hazmat_queue.get_nowait()
            last_hazmat_update = time.time()
            if state_hazmat["hazmat_frame"] is not None:
                hazmat_frame = state_hazmat["hazmat_frame"]
                hazmat_frame_orginal = hazmat_frame.copy()
            else:
                hazmat_frame = np.zeros_like(frame)
        except:
            if hazmat_frame_orginal is not None:
                hazmat_frame = hazmat_frame_orginal.copy()
            else:
                hazmat_frame = np.zeros_like(frame)

        frame_to_pass_to_hazmat = frame.copy()


        state_main["run_hazmat"] = run_hazmat.get()

        fps = -1 if delta == 0 else 1 / delta
        hazmat_fps = min(-1 if state_hazmat["hazmat_delta"] == 0 else 1 / state_hazmat["hazmat_delta"], 100)

        if debug:
            print(f"FPS: {fps:.0f}\tHazmat FPS: {hazmat_fps:.0f}\tHazmat: {run_hazmat}\tQR: {run_qr}")

        if run_qr:
            start = time.time()

            qr_found_this_frame = qr_detect(frame)
            if len(qr_found_this_frame) > 0:
                for qr in qr_found_this_frame:
                    all_qr_found.append(qr)
                all_qr_found = list(set(all_qr_found))

                print(qr_found_this_frame)
                print(all_qr_found)

            end = time.time()

            ratio = min((end - start) / QR_TIME_BAR_SCALE, 1)
            w = ratio * (frame.shape[1] - 10)

            cv2.line(
                frame,
                (5, 5),
                (5 + int(w), 5),
                (0, 0, 255),
                3
            )
        else:
            cv2.line(
                frame,
                (5, 5),
                (5, 5),
                (255, 255, 0),
                3
            )

        t1 = time.time()
        delta = t1 - t0
        t0 = t1

        # fps text (bottom left)
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (5, frame.shape[0] - 5)
        fontScale              = 0.5
        fontColor              = (0, 255, 0)
        thickness              = 1
        lineType               = 2

        text                   = "FPS: %.0f" % fps
        cv2.putText(frame, text, bottomLeftCornerOfText, font, fontScale, fontColor, thickness, lineType)

        text                  = "Hazmat FPS: %.0f" % hazmat_fps
        cv2.putText(hazmat_frame, text, bottomLeftCornerOfText, font, fontScale, fontColor, thickness, lineType)

        time_since_last_hazmat_update = time.time() - last_hazmat_update
        ratio = min(time_since_last_hazmat_update / HAZMAT_DELAY_BAR_SCALE, 1)
        w = ratio * (frame.shape[1] - 10)

        cv2.line(
            hazmat_frame,
            (5, 5),
            (5 + int(w), 5),
            (255, 255, 0) if not run_hazmat.get() else (0, 0, 255),
            3
        )


        top_combined = cv2.hconcat([frame, hazmat_frame])
        if video_capture_zero:
            bottom_combined = cv2.hconcat([frames["webcam1"], ir_frame])
        else:
            bottom_combined = cv2.hconcat([frames["webcam2"], ir_frame])

        combined = cv2.vconcat([top_combined, bottom_combined])


        # cv2.imshow("Camera and hazmat", combined)

        # key = cv2.waitKey(1) & 0xFF

        old_server_state = SERVER_STATE.copy()
        read_state()

        for key, value in SERVER_STATE.items():
            if key not in old_server_state or old_server_state[key] != value:
                if value:
                    if key == QUIT_KEY:
                        state_main["quit"] = True
                        return caps
                    if key == HAZMAT_TOGGLE_KEY:
                        run_hazmat.toggle()
                    if key == QR_TOGGLE_KEY:
                        run_qr.toggle()
                    if key == QR_CLEAR_KEY:
                        all_qr_found = []
                    if key == HAZMAT_CLEAR_KEY:
                        state_main["clear_all_found"] = 1

                    # SERVER_STATE[key] = False
                    

        if state_main["clear_all_found"] == 1:
            state_main["clear_all_found"] = 2
        elif state_main["clear_all_found"] == 2:
            state_main["clear_all_found"] = 0

        state_main["frame"] = frame_to_pass_to_hazmat
        main_queue.put_nowait(state_main)

        combine_downscaled = cv2.resize(combined, (0, 0), fx=SERVER_FRAME_SCALE, fy=SERVER_FRAME_SCALE)
        MAIN_STATE["frame"] = base64.b64encode(cv2.imencode('.jpg', combine_downscaled)[1]).decode()

        MAIN_STATE["w"] = combine_downscaled.shape[1]
        MAIN_STATE["h"] = combine_downscaled.shape[0]

        write_state()


ap = argparse.ArgumentParser()
ap.add_argument("-d", "--debug", required=False, help="show debug prints", action="store_true")
ap.add_argument("-z", "--video-capture-zero", required=False, help="use VideoCapture(0)", action="store_true")
args = vars(ap.parse_args())

if __name__ == "__main__":
    print("Starting hazmat thread...")

    main_queue = Queue()
    hazmat_queue = Queue()

    hazmat_thread = Process(target=hazmat_main, args=(main_queue, hazmat_queue))

    hazmat_thread.start()

    print("Starting main thread...")
    caps = main(main_queue, hazmat_queue, args["debug"], args["video_capture_zero"])

    for cap in caps.values():
        cap.release()
    cv2.destroyAllWindows()

    print("Exiting...")

    START_STATE_MAIN["quit"] = True
    main_queue.put_nowait(START_STATE_MAIN)
    
    hazmat_thread.terminate()
    time.sleep(1) # wait for thread to terminate
    hazmat_thread.close()

    print("Closing queues...")

    main_queue.close()
    hazmat_queue.close()

    # basically `allow_exit_without_flush`
    main_queue.cancel_join_thread()
    hazmat_queue.cancel_join_thread()

    print("Done.")
