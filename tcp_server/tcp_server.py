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

        match data[0]:
            case 0:
                
            case 1:
                x = (data[1] << 8) + data[2]
                y = (data[3] << 8) + data[4]
                angle = (data[5] << 8) + data[6]
                set_goal_position(x,y, angle)
                state = States.POSITIONING
            case 2:
                #nastavit rychlost (velocity)
                x = (data[1] << 8) + data[2]
                y = (data[3] << 8) + data[4]
                set_velocity(x,y)
                state = States.VEL_CONTROL
            case 3:
                state = States.CALIBRATION
            case 4:
                #stop
                state = States.STOP
            case _:



def main_loop()
    while True:
        match state:
            case States.POSITIONING:
                send_actual_position(0,0,0,0)
            case States.VEL_CONTROL:
                send_actual_position(0,0,0,0)
            case States.STOP:
            case States.CALIBRATION:
                start_calibration_process()



def set_goal_position(x, y, angle)
def set_velocity(x,y)
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
def start_calibration_process()
    while(true)
        #proces kalibrace



            


