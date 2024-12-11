'''

Author: Nicholas Graham (ngraham32@gatech.edu) (nickgraham654@gmail.com)
Description: 
Middleman code of handlers between GUI.py and (pymavlink\server\controller.py)

'''

from GUI_port import send, recieve

def on_rtl():
    print("rtl button pressed")
    send("rtl\n")

def on_land():
    print("Land button pressed")
    send("land\n")

def on_takeoff():
    print("Takeoff button pressed")
    send("takeoff\n")

def on_up():
    print("Up button pressed")
    send("up\n")

def on_down():
    print("Down button pressed")
    send("down\n")

def on_left():
    print("Left button pressed")
    send("left\n")

def on_right():
    print("Right button pressed")
    send("right\n")

def on_drone_loc():
    print("Drone Location button pressed")
    send("drone_loc\n")
    msg = recieve()
    if msg: 
        print(msg)

def on_drone_takeoff():
    print("Drone Takeoff button pressed")
    send("takeoff\n")

def on_drone_land():
    print("Drone Land button pressed")
    send("takeoff\n")

def on_drone_init():
    print("Drone init signaled")
    recieve()
