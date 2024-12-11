## Main Files of Interest:

- `pymavlink\client\GUI.py` - Client Application GUI 
- `pymavlink\server\controller.py` - Backend Drone Controller Server
- `pymavlink\server\Object_Detect\PeopleFind.py` - Backend Object Detection Server


## Installation (Windows):

1. **Ensure that Python, pip, Visual Studio Code, and Nvidia cuda is installed on your machine.**
  - 1.1 Ensure that the Python and Python Debugger extensions are installed in VS Code 
2 **Create a Visual Studio Project with a venv pip virtual environment**
  - 2.1 Press Ctrl+shift+P
  - 2.2 Python: Create Environment
  - 2.3 Venv
  - 2.4 Python 3.8.3 64-bit
3 **Inside of this virtual environment, install dependencies using the following commands:**
      ```bash
        pip install imutils==0.5.4
        pip install numpy>=1.23.0
        pip install opencv-python==4.10.0.84
        pip install Pillow==9.0.1
        pip install pymavlink==2.4.42
        pip install ultralytics==8.3.26 #Note: This takes a long time
      ```
## Run:

Run in the following order:

1 **Go to (pymavlink\server\Object_Detect\PeopleFind.py) and run in VScode (F5)** 
  - 1.1 Ensure that a camera input for the drone camera feed is detected in windows device manager
  - 1.2 Wait for this prompt or similar before continuing: 
     ```
     Using GPU
     HOST IP: localhost
     LISTENING AT: ('localhost', 9998)
     ```

2 **Go to (pymavlink\server\controller.py) and also run in VScode (F5)** 
  - 2.1 Ensure that your machine has access to the drone's flight controller either through telemetry COM port or via Network
    - 2.1.1 Depending on the specific COM port, you may need to change the configured COM port used in Line 42 of controller.py

3 **Go to (pymavlink\client\GUI.py) and also run in VScode (F5)**
  - 3.1 This will open the GUI, which you can use to command the drone and recieve video feed.
