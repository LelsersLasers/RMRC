cap_args = "v4l2src device=/dev/v4l/by-id/usb-046d_C270_HD_WEBCAM_2D4AA0A0-video-index0 ! videoconvert ! video/x-raw,format=UYVY ! videoscale ! video/x-raw,width=320,height=240 ! videoconvert ! appsink"


import cv2
import time
from levenshtein import *
from cleanUp import *
import numpy as np
import argparse
import mahotas
import pytesseract
# import threading
from multiprocessing import Process, Queue


QUIT_KEY = "q"
TOGGLE_KEY = "g"
CLEAR_KEY = "c"

CAMERA_WAKEUP_TIME = 1

# What main thread sends
START_STATE_MAIN = {
    "frame": None,
    "run_hazmat": False,
    "quit": False,
    "clear_all_found": False,
}

# What hazmat thread sends
START_STATE_HAZMAT = {
    "hazmat_delta": 1 / 10,
    "hazmat_frame": None,
    # "cleared_all_found": False,
}

ap = argparse.ArgumentParser()
ap.add_argument("-d", "--debug", required=False, help="show debug windows", action="store_true")
args = vars(ap.parse_args())


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

    if args["debug"]:
        cv2.imshow("Inverted Binary Image", img)

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

                    if args["debug"]:
                        cv2.imshow("squares", img)

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
        if args["debug"]:
            cv2.rectangle(image, (x1, y1), (x2, y2), (225, 0, 0), 2)
            cv2.imshow(f"image {i}", onlyText)
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


# ENUM
class Mode:
    Normal = 0
    Hazmat = 1

    def toggle(state):
        if state == Mode.Normal:
            return Mode.Hazmat
        elif state == Mode.Hazmat:
            return Mode.Normal
        else:
            raise ValueError("Invalid mode")

    def to_str(state):
        if state == Mode.Normal:
            return "Normal"
        elif state == Mode.Hazmat:
            return "Hazmat"
        else:
            raise ValueError("Invalid mode")


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

        try:
            state_main = main_queue.get_nowait()
        except:
            pass

        if state_main["frame"] is not None:
            frame = state_main["frame"]

            if state_main["run_hazmat"]:

                if state_main["clear_all_found"]:
                    all_found = []
                    # state["clear_all_found"] = False # TODO

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

                print([x[0] for x in found_this_frame])
                print(all_found)

            state_hazmat["hazmat_frame"] = frame

        t1 = time.time()
        delta = t1 - t0
        t0 = t1

        sleep_time = max(0.1 - delta, 0)
        time.sleep(sleep_time)

        state_hazmat["hazmat_delta"] = delta

        hazmat_queue.put_nowait(state_hazmat)


def main(main_queue, hazmat_queue):
    print("Starting camera...")

    cap = cv2.VideoCapture(cap_args, cv2.CAP_GSTREAMER)
    # cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        raise RuntimeError(
            "Can't open camera. Are the cap_args set right? Is the camera plugged in?"
        )


    time.sleep(CAMERA_WAKEUP_TIME)


    print(f"\nPress '{QUIT_KEY}' to quit.")
    print(f"Press '{TOGGLE_KEY}' to toggle running hazmat detection.")
    print(f"Press '{CLEAR_KEY}' to clear all found hazmat labels.\n")

    t0 = time.time()
    t1 = time.time()
    delta = 1 / 30

    mode = Mode.Normal

    state_main = START_STATE_MAIN
    state_hazmat = START_STATE_HAZMAT

    last_hazmat_update = time.time()

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            print("Exiting ...")
            break

        try:
            state_hazmat = hazmat_queue.get_nowait()
            last_hazmat_update = time.time()
        except:
            pass

        state_main["frame"] = frame

        state_main["run_hazmat"] = mode == Mode.Hazmat

        fps = -1 if delta == 0 else 1 / delta
        hazmat_fps = min(-1 if state_hazmat["hazmat_delta"] == 0 else 1 / state_hazmat["hazmat_delta"], 100)
        print(f"FPS: {fps:.1f}\tHazmat FPS: {hazmat_fps:.1f}\t(Mode: {Mode.to_str(mode)})")

        t1 = time.time()
        delta = t1 - t0
        t0 = t1

        hazmat_frame = state_hazmat["hazmat_frame"] if state_hazmat["hazmat_frame"] is not None else np.zeros_like(frame)

        # fps text (bottom left)
        font                   = cv2.FONT_HERSHEY_SIMPLEX
        bottomLeftCornerOfText = (5, frame.shape[0] - 5)
        fontScale              = 0.5
        fontColor              = (0, 255, 0)
        thickness              = 1
        lineType               = 2

        text                   = "FPS: %.1f" % fps
        cv2.putText(frame, text, bottomLeftCornerOfText, font, fontScale, fontColor, thickness, lineType)

        text                  = "Hazmat FPS: %.1f" % hazmat_fps
        cv2.putText(hazmat_frame, text, bottomLeftCornerOfText, font, fontScale, fontColor, thickness, lineType)

        time_since_last_hazmat_update = time.time() - last_hazmat_update
        start_angle = time_since_last_hazmat_update * 360
        end_angle = start_angle + 60
        cv2.ellipse(
            hazmat_frame,
            (15, 15),
            (10, 10),
            0,
            start_angle,
            end_angle,
            (255, 255, 0),
            2
        )
        

        frame_combined = cv2.hconcat([frame, hazmat_frame])


        cv2.imshow("Camera and hazmat", frame_combined)

        key = cv2.waitKey(1) & 0xFF
        if key == ord(QUIT_KEY):
            state_main["quit"] = True
            break
        elif key == ord(TOGGLE_KEY):
            mode = Mode.toggle(mode)
        elif key == ord(CLEAR_KEY):
            state_main["clear_all_found"] = True

        main_queue.put_nowait(state_main)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    print("Starting hazmat thread...")

    main_queue = Queue()
    hazmat_queue = Queue()

    hazmat_thread = Process(target=hazmat_main, args=(main_queue, hazmat_queue))

    hazmat_thread.start()

    print("Starting main thread...")
    main(main_queue, hazmat_queue)

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
