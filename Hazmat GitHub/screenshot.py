import cv2

def screenshot():
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Can't open camera.")
        quit()
    else:
        print("Opened camera.")

    print("Press <Enter> when you are ready.")
    input()
    print("Click 'g' when satisfied with the image.")

    key = ""
    while key != ord("g"):
        _, frame = cap.read()
        cv2.imshow("image", frame)
        key = cv2.waitKey(1)

    print("All done gathering images.")
    # cv2.imshow("test", frame)

    output_path = "/Users/AudreyLin/Desktop/images/new_image.jpg"  # Specify the desired output path and filename
    cv2.imwrite(output_path, frame)
    print(f"Saved the final image as {output_path}")
    return frame

