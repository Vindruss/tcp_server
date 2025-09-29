# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
import threading
import socket
from enum import Enum

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry 

from std_msgs.msg import Header


from rclpy.node import Node

from std_msgs.msg import String

class States(Enum):
    POSITIONING = 1
    VEL_CONTROL = 2
    CALIBRATION = 3
    STOP = 4


class MinimalPublisher(Node):
    goal_linear_velocity_x = 0.0
    goal_linear_velocity_y = 0.0
    goal_angular_velocity_z = 0.0
    goal_position_x = 0.0
    goal_position_y = 0.0
    goal_angle = 0.0
    actual_linear_velocity_x = 0.0
    actual_linear_velocity_y = 0.0
    actual_angular_velocity_z = 0.0
    actual_position_x = 0.0
    actual_position_y = 0.0
    actual_angle = 0.0
    state = States.STOP
    conn_state = False
    conn = None
    def __init__(self):
        super().__init__('tcp_server')
        #self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel_out', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        


        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback_odom,
            10)
        self.subscription_odom  

        t1 = threading.Thread(target=self.tcp_loop, args=())
        t1.start()

        

    # naslouchani aktualni pozice
    def listener_callback_odom(self, msg):     
        self.actual_position_x = msg.pose.pose.position.x
        self.actual_position_y = msg.pose.pose.position.y
        self.actual_angle= msg.pose.pose.orientation.z
        self.actual_angular_velocity_z = msg.twist.twist.angular.z
        self.actual_linear_velocity_x = msg.twist.twist.linear.x
        self.actual_linear_velocity_y = msg.twist.twist.linear.y
        

    #   
        
        


    # posílání goal position a rychlosti   
    def timer_callback(self): 
        if not self.conn_state:
            return
        print(f"actual pos: {self.actual_position_x} {self.actual_position_y} {self.actual_angle}")
        actual_position_x_bytes = int(self.actual_position_x).to_bytes(2, 'big')
        actual_position_y_bytes = int(self.actual_position_y).to_bytes(2, 'big')
        actual_linear_velocity_x_bytes = int(self.actual_linear_velocity_x).to_bytes(2, 'big')
        actual_linear_velocity_y_bytes = int(self.actual_linear_velocity_y).to_bytes(2, 'big')
        actual_angular_velocity_z_bytes = int(self.actual_angular_velocity_z).to_bytes(2, 'big')
        actual_angle_bytes = int(self.actual_angle).to_bytes(2, 'big')
        message = [101, actual_position_x_bytes[0], actual_position_x_bytes[1], actual_position_y_bytes[0], actual_position_y_bytes[1], actual_angle_bytes[0], actual_angle_bytes[1],
                   actual_linear_velocity_x_bytes[0], actual_linear_velocity_x_bytes[1], actual_linear_velocity_y_bytes[0], actual_linear_velocity_y_bytes[1],
                   actual_angular_velocity_z_bytes[0], actual_angular_velocity_z_bytes[1]]
        self.conn.send((bytes(message)))
        
        
        

        
    def tcp_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = '10.0.100.145'
        port = 8899
        print(f"TCP socket sonn.rectarted")
        s.bind((host, port))
        s.listen()
        self.conn, addr = s.accept()
        print(f"Connected by {addr}")
        self.conn_state = True
        self.conn.sendall(b'Connected to TCP server\r\n')
        while True:
            data = self.conn.recv(1024)
            if not data:
                break
                
            
            # if data[0] == 97:
            #     conn.send(('Position: ' + str(self.x) + ' ' + str(self.y)).encode())
            #     #self.velocity = 1.0
            # else: 
            #     conn.send(b"spatne")
            #     #self.velocity = 0.0

            #urceni typu zpravy
            match data[0]:
                case 0:
                    pass
                case 1:
                    self.goal_position_x = (data[1] << 8) + data[2]
                    self.goal_position_y = (data[3] << 8) + data[4]
                    self.goal_angle = (data[5] << 8) + data[6]                    
                    self.set_goal_position(self.goal_position_x,self.goal_position_y, self.goal_angle)
                    #state = States.POSITIONING
                    #2 - jed urcitou rychlosti
                case 2:
                    #nastavit rychlost (velocity)
                    self.goal_linear_velocity_x = (data[1] << 8) + data[2]
                    self.goal_linear_velocity_y = (data[3] << 8) + data[4]
                    self.set_velocity(self.goal_linear_velocity_x,self.goal_linear_velocity_y)
                    #state = States.VEL_CONTROL

                #3 - zahaj kalibraci 
                case 3:
                    self.actual_angular_velocity_z = (data[1] << 8) + data[2]
                    self.set_angular_velocity(self.actual_angular_velocity_z)
                #4 - zastav vozitko
                case 4:
                    self.stop_rover()
                    #state = States.STOP

                case _:
                    pass
                

    def set_goal_position(x, y, angle):
        msg = Pose()
        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.orientation.z = float(angle)
        self.publisher.publish(msg)
        
    def set_velocity(x,y):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        self.publisher.publish(msg)

    def set_angular_velocity(z):
        msg = Twist()
        msg.angular.z = float(z)
        self.publisher.publish(msg)

    def stop_rover():
        msg = Twist()
        msg.linear.x = float(0)
        msg.linear.y = float(0)
        msg.angular.z = float(0)
        self.publisher.publish(msg)
        



def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
