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
current_dir = os.getcwd()  # Check the current working directory

DELAY = 1
video_tx = False
video_path = 'crowd.mp4'
video_path = os.path.join(current_dir, 'crowd.mp4')
#Camera
#cap = cv2.VideoCapture(video_path)
cap = cv2.VideoCapture(0)

#Code attributed from: https://core-electronics.com.au/guides/raspberry-pi/getting-started-with-yolo-object-and-animal-recognition-on-the-raspberry-pi/
#Code attributed from: https://github.com/techwithtim/OpenCV-Tutorials/blob/main/tutorial8.py

model = YOLO("yolov10n.pt")

'''
if not cap.isOpened():
    print("Error: Could not open video file.")
    exit()
'''

if video_tx:
    init_vid_tx()

# Set to keep track of detected person IDs (if YOLO provides unique IDs, e.g., via object tracking)
detected_person_ids = set()

# Time of Previous Model Use
last_time = time.time()
#frame = cv2.resize(frame, (640, 480))

annotated_frame = None

while True:
    # Capture a frame from the camera
    ret, frame = cap.read()

    current_time = time.time()
    if current_time - last_time >= DELAY:
        last_time = current_time

        # Run YOLO model on the captured frame and store the results
        results = model.track(frame, classes=0)
        #results = model(frame, classes=0)
                
        if len(results) > 0:
            print(f"Person(s) detected! {len(results[0])}")
        else:
            print("No person detected in this frame.")

        detections = results[0].boxes  # Get bounding box results

        # Extract bounding boxes
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())  # Get bounding box coordinates
            cropped_image = frame[y1:y2, x1:x2]  # Crop the bounding box area

            # Display the cropped image
            cv2.imshow("Cropped Image", cropped_image)

            # Break if 'q' is pressed
            if cv2.waitKey(1) == ord("q"):
                break


        # Iterate through each detection to track new persons
        for box in detections:
            obj_id = box.id if hasattr(box, 'id') else None  # Get unique ID if available
            if obj_id is not None:
                # Convert tensor ID to a plain type (e.g., integer or string)
                obj_id = obj_id.item() if obj_id.numel() == 1 else tuple(obj_id.tolist())
                
                if obj_id not in detected_person_ids:  # Check for new person
                    detected_person_ids.add(obj_id)  # Add new person ID to the set
                    print(f"New person detected! ID: {obj_id}")
        
        ''' 
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
        '''
    if video_tx:
        vid_2_client(annotated_frame)
    '''
    else:
        time.sleep(1)
    '''
    
    # Exit the program if q is pressed
    if cv2.waitKey(1) == ord("q"):
        break

cap.release()
# Close all windows
cv2.destroyAllWindows()
server_socket.close()