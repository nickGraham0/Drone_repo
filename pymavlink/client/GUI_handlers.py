from GUI_port import send, recieve


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
        #save_to_csv(msg)

def on_drone_takeoff():
    print("Drone Takeoff button pressed")
    send("takeoff\n")

def on_drone_land():
    print("Drone Land button pressed")
    send("takeoff\n")

def on_drone_init():
    print("Drone init signaled")
    recieve()
    #Drone Init Light turns on