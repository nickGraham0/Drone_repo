#Code attributed from: https://core-electronics.com.au/guides/raspberry-pi/getting-started-with-yolo-object-and-animal-recognition-on-the-raspberry-pi/
#Code attributed from: https://github.com/techwithtim/OpenCV-Tutorials/blob/main/tutorial8.py

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
CONFIDENCE = 0.8
PERSON = 0

video_tx = True
cap = cv2.VideoCapture(0)
#cap = cv2.VideoCapture('crowd.mp4')


model = YOLO("yolov10x.pt")
if not torch.cuda.is_available():
    model.to('cpu')
else:
    model.to('cuda')




if video_tx:
    init_vid_tx()

detected_person_ids = set()

last_time = time.time()

annotated_frame = None

while True:
    ret, frame = cap.read()
    #frame = cv2.resize(frame, (640, 480))

    current_time = time.time()
    if current_time - last_time >= DELAY:
        last_time = current_time

        results = model.track(frame, classes=PERSON, conf=CONFIDENCE)
                
        detections = results[0].boxes  # Get bounding box results

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
server_socket.close()