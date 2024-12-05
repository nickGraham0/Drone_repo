# Import required libraries
import cv2

#attributed code from https://thedatafrog.com/en/articles/human-detection-video/
#https://thedatafrog.com/en/articles/human-detection-video/
#found to be too slow to support detection (not utilizing GPU)

# Path to the video file
video_path = 'pymavlink/people.mp4'
cap = cv2.VideoCapture(video_path)

# Initialize the HOG descriptor and SVM detector
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

if not cap.isOpened():
    print("Error: Could not open video file.")
    exit()

while True:
    # Capture each frame from the video
    ret, frame = cap.read()
    if not ret:
        print("End of video or error in capturing frame.")
        break

    # Detect humans in the current frame
    (humans, _) = hog.detectMultiScale(frame, winStride=(10, 10), padding=(32, 32), scale=1.1)

    # Print number of humans detected in the frame
    print('Human Detected:', len(humans))

    # Loop over all detected humans and draw bounding boxes
    for (x, y, w, h) in humans:
        pad_w, pad_h = int(0.15 * w), int(0.01 * h)
        cv2.rectangle(frame, (x + pad_w, y + pad_h), (x + w - pad_w, y + h - pad_h), (0, 255, 0), 2, cv2.LINE_AA)

    # Display the resulting frame
    cv2.imshow("Camera", frame)

    # Exit the program if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

# Release resources and close windows
cap.release()
cv2.destroyAllWindows()
