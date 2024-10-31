import cv2
import numpy
import socket 
import pickle 
import struct 
from vid_tx import init_vid_tx, vid_2_client
from ultralytics import YOLO

video_path = 'crowd.mp4'

#Camera
cap = cv2.VideoCapture(video_path)

#Code attributed from: https://core-electronics.com.au/guides/raspberry-pi/getting-started-with-yolo-object-and-animal-recognition-on-the-raspberry-pi/
#Code attributed from: https://github.com/techwithtim/OpenCV-Tutorials/blob/main/tutorial8.py

model = YOLO("yolov10x.pt")

if not cap.isOpened():
    print("Error: Could not open video file.")
    exit()

init_vid_tx()

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()
    #frame = cv2.resize(frame, (640, 480))

    # Run YOLO model on the captured frame and store the results
    #results = model.track(frame, classes=0)
    results = model(frame, classes=0)
    
    annotated_frame = results[0].plot()
    
    inference_time = results[0].speed['inference']
    fps = 1000 / inference_time  # Convert to milliseconds
    text = f'FPS: {fps:.1f}'

    font = cv2.FONT_HERSHEY_SIMPLEX
    text_size = cv2.getTextSize(text, font, 1, 2)[0]
    text_x = annotated_frame.shape[1] - text_size[0] - 10  # 10 pixels from the right
    text_y = text_size[1] + 10  # 10 pixels from the top

    cv2.putText(annotated_frame, text, (text_x, text_y), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

    cv2.imshow("Camera", annotated_frame)

    vid_2_client(annotated_frame)
    
    # Exit the program if q is pressed
    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
# Close all windows
cv2.destroyAllWindows()
server_socket.close()