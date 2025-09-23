import socket
from enum import Enum

class States(Enum):
    POSITIONING = 1
    VEL_CONTROL = 2
    CALIBRATION = 3
    STOP = 4

HOST = "192.168.0.10"  
PORT = 65432 

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

def main()
    s.bind((HOST, PORT))
    s.listen()
    conn, addr = s.accept()

    print(f"Connected by {addr}")
    while True:
        data = conn.recv(1024)
        if not data:
            break
        #urceni typu zpravy
        match data[0]:
            case 0:
                
            #1 - jed na pozici 
            case 1:
                x = (data[1] << 8) + data[2]
                y = (data[3] << 8) + data[4]
                angle = (data[5] << 8) + data[6]
                set_goal_position(x,y, angle)
                state = States.POSITIONING
            #2 - jed urcitou rychlosti
            case 2:
                #nastavit rychlost (velocity)
                x = (data[1] << 8) + data[2]
                y = (data[3] << 8) + data[4]
                set_velocity(x,y)
                state = States.VEL_CONTROL

            #3 - zahaj kalibraci 
            case 3:
                #zahajeni kalibrace
                state = States.CALIBRATION
            #4 - zastav vozitko
            case 4:
                stop_rover()
                state = States.STOP
            #101 - posli aktualni pozici
            case 101:
                send_actual_position(0,0,0,0)

            case _:



def main_loop()
    while True:
        match state:
            case States.POSITIONING:
                #čekání na to až dojede vozitko do pozice
                #mezitim posílá aktualní polohu 
                send_actual_position(0,0,0,0)
            case States.VEL_CONTROL:
                send_actual_position(0,0,0,0)
            case States.STOP:
                
            case States.CALIBRATION:
                start_calibration_process()



def set_goal_position(x, y, angle)
    #zaslani topicu goal_position
def set_velocity(x,y)
    #zaslani topicu cmd_vel

def send_actual_position(x, y, vel, angle)
    x_bytes = x.to_bytes(2, "big")  
    y_bytes = y.to_bytes(2, "big")
    angle_bytes = angle.to_bytes(2, "big")
    message_list = [11,x_bytes[0], x_bytes[1], y_bytes[0], y_bytes[1], angle_bytes[0], angle_bytes[1]]
    message = bytes(message_list) 
    s.sendall(message) 

def send_calibration_status(status)
    x_bytes = x.to_bytes(2, "big")  
    y_bytes = y.to_bytes(2, "big")
    angle_bytes = angle.to_bytes(2, "big")
    message_list = [11,x_bytes[0], x_bytes[1], y_bytes[0], y_bytes[1], angle_bytes[0], angle_bytes[1]]
    message = bytes(message_list) 
    s.sendall(message) 

def stop_rover()
    set_velocity(0,0)
def start_calibration_process()
    while(true)
        #proces kalibrace



            


