'''

Author: Nicholas Graham (ngraham32@gatech.edu) (nickgraham654@gmail.com)
Code Attributed from: 
    https://core-electronics.com.au/guides/raspberry-pi/getting-started-with-yolo-object-and-animal-recognition-on-the-raspberry-pi/
    https://github.com/techwithtim/OpenCV-Tutorials/blob/main/tutorial8.py
Description: 
Object Detection/Tracking of Human shapes detected in supplied video feed. Uses YOLOv10 pretrained object detection model to identify human shapes and track individual detections
over multiple frames. Uses vid_tx.py to send a new, resulting video feed that is annotated with bounding boxes to (pymavlink\client\GUI.py)

'''

import cv2
import numpy
import socket 
import pickle 
import struct 
from vid_tx import init_vid_tx, vid_2_client
from ultralytics import YOLO
from ultralytics.engine.results import Results
import time
import os
import torch 

current_dir = os.getcwd()  # Check the current working directory

VIDEO = 0

DELAY = 0
CONFIDENCE = 0.4
PERSON = 0

video_tx = True
cap = cv2.VideoCapture(0)

#If using .mp4 file instead of camera in
#cap = cv2.VideoCapture('crowd.mp4')

model = YOLO("yolov10n.pt")
if not torch.cuda.is_available():
    print('Using CPU')
    model.to('cpu')
else:
    print('Using GPU')
    model.to('cuda')



# True if sending video stream to GUI. Else, false.
if video_tx:
    init_vid_tx()

detected_person_ids = set()

last_time = time.time()

annotated_frame = None

'''

Repeatedly retrieve frame from camera in, feed it into the YOLO model, and then annotate and output the result.

'''
while True:
    #Read from Camera In
    ret, frame = cap.read()
    frame = cv2.resize(frame, (640, 480))

    current_time = time.time()
    if current_time - last_time >= DELAY:
        last_time = current_time

        #Feed into YOLO Model. Retreive Detections
        results = model.track(frame, classes=PERSON, conf=CONFIDENCE)
        detections = results[0].boxes  

    # Output a snippet of each detection
    for box in results[0].boxes:
        obj_id = box.id if hasattr(box, 'id') else None  # Check if 'id' attribute exists
        if obj_id is not None:
            obj_id = obj_id.item() if obj_id.numel() == 1 else tuple(obj_id.tolist())
            
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

            cropped_image = frame[y1:y2, x1:x2]

            
            if video_tx:
                vid_2_client(cropped_image, obj_id)
            else:
                cv2.imshow(f"New Person ID: {obj_id}", cropped_image)

        else:
            continue

        annotated_frame = results[0].plot()
        
        inference_time = results[0].speed['inference']
        fps = 1000 / inference_time  
        text = f'FPS: {fps:.1f}'

        font = cv2.FONT_HERSHEY_SIMPLEX
        text_size = cv2.getTextSize(text, font, 1, 2)[0]
        text_x = annotated_frame.shape[1] - text_size[0] - 10  # 10 pixels from the right
        text_y = text_size[1] + 10  # 10 pixels from the top

        cv2.putText(annotated_frame, text, (text_x, text_y), font, 1, (255, 255, 255), 2, cv2.LINE_AA)

    # Output the entirety of the annotated frame
    if len(results[0]) > 0:
        if video_tx:
            vid_2_client(annotated_frame, VIDEO)
        else:
            cv2.imshow("Camera", annotated_frame)
    else:
        if video_tx:
            vid_2_client(frame, VIDEO)
        else:
            cv2.imshow("Camera", frame)

    
    # Exit the program if q is pressed
    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
# Close all windows
cv2.destroyAllWindows()
