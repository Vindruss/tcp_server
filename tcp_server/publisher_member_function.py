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

from multiprocessing import process
import rclpy
import threading
import socket
from enum import Enum

import sys

from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, TimerAction
from launch import LaunchIntrospector  # noqa: E402
from launch import LaunchService  # noqa: E402
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry 
from rclpy.action import ActionServer
from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import OccupancyGrid
from tf_transformations import euler_from_quaternion
import launch_ros.actions  # noqa: E402

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
        self.publisher_twist = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_pose = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.publisher_initial_pose = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)

        

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        
        


        self.subscription_odom = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback_odom,
            10)
        self.subscription_odom  

        self.subscription_goal_status = self.create_subscription(
            GoalStatusArray,
            'navigate_to_pose/_action/status',
            self.listener_callback_goal_status,
            10)
        self.subscription_goal_status

        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            'map',
            self.listener_callback_map,
            10)
        self.subscription_map


        ld = LaunchDescription([
        launch_ros.actions.Node(
            package='demo_nodes_cpp', executable='talker', output='screen',
            remappings=[('chatter', 'my_chatter')]),
        launch_ros.actions.Node(
            package='demo_nodes_cpp', executable='listener', output='screen',
            remappings=[('chatter', 'my_chatter')]),
        ])

        self.launch_service = LaunchService()
        self.launch_service.include_launch_description(ld)
        self.launch_service.start()


        t1 = threading.Thread(target=self.tcp_loop, args=())
        t1.start()

    def listener_callback_map(self, msg):
        
        #print(f"actual pos: {self.actual_position_x} {self.actual_position_y} {self.actual_angle}")
        map_resolution_bytes = int(msg.info.resolution*1000).to_bytes( 4 , byteorder='little' , signed=True )
        map_width_bytes = int(msg.info.width).to_bytes( 4 , byteorder='little' , signed=True )
        map_height_bytes = int(msg.info.height).to_bytes( 4 , byteorder='little' , signed=True )
        map_origin_x_bytes = int(msg.info.origin.position.x*1000).to_bytes( 4 , byteorder='little' , signed=True )
        map_origin_y_bytes = int(msg.info.origin.position.y*1000).to_bytes( 4 , byteorder='little' , signed=True )
        

        # convert msg.data to signed bytes
        signed_data = []
        for i in msg.data:
            if i < 0:
                signed_data.append(256 + i)
            else:
                signed_data.append(i)
            
                
            

        print(f"Map: {msg.info.resolution} {msg.info.width} {msg.info.height} {msg.info.origin.position.x} {msg.info.origin.position.y} {len(msg.data)}")
        message = [103] + list(map_resolution_bytes) + list(map_width_bytes) + list(map_height_bytes) + list(map_origin_x_bytes) + list(map_origin_y_bytes)
        if not self.conn_state:
            return
        # send header
        self.conn.sendall((bytes(message)))   

        #send obstacle data as points
        # for x in range(msg.info.width):
        #     for y in range(msg.info.height):
        #         index = x + y * msg.info.width
        #         if msg.data[index] > 50:
        #             point_x_bytes = int(x).to_bytes( 4 , byteorder='little' , signed=True )
        #             point_y_bytes = int(y).to_bytes( 4 , byteorder='little' , signed=True )
        #             point_message = [104] + list(point_x_bytes) + list(point_y_bytes)
        #             self.conn.sendall((bytes(point_message)))

        #send map data in chunks of 1024 bytes
        chunk_size = 500
        for i in range(0, len(signed_data), chunk_size):
            if(i + chunk_size > len(signed_data)):
                chunk_size = len(signed_data) - i
            chunk = [104] + signed_data[i:i+chunk_size]
            self.conn.sendall((bytes(chunk)))

      
            

            

        

    # naslouchani aktualni pozice
    def listener_callback_odom(self, msg):     
        self.actual_position_x = msg.pose.pose.position.x
        self.actual_position_y = msg.pose.pose.position.y
        self.actual_angular_velocity_z = msg.twist.twist.angular.z
        self.actual_linear_velocity_x = msg.twist.twist.linear.x
        self.actual_linear_velocity_y = msg.twist.twist.linear.y

        quaternion_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)
        self.actual_angle = yaw

        
    def listener_callback_goal_status(self, msg):
        if len(msg.status_list) == 0:
            return
        match msg.status_list[-1].status:
            case 1:
                print("GOAL ACCEPTED")
            case 2:
                print("GOAL IS EXECUTING")
            case 3:
                print("GOAL IS CANCELING")
            case 4:
                print("GOAL SUCCEDED")
            case 5:
                print("GOAL ABORTED")
            case 6:
                print("GOAL CANCELED")

            case _:
                pass

        if not self.conn_state:
            return
        #print(f"actual pos: {self.actual_position_x} {self.actual_position_y} {self.actual_angle}")
        
        message = [102] + [msg.status_list[-1].status] 
        self.conn.sendall((bytes(message)))
        
    #   
        
        


    # posílání goal position a rychlosti   
    def timer_callback(self): 
        if not self.conn_state:
            return
        #print(f"actual pos: {self.actual_position_x} {self.actual_position_y} {self.actual_angle}")
        actual_position_x_bytes = int(self.actual_position_x*1000).to_bytes( 4 , byteorder='little' , signed=True )
        actual_position_y_bytes = int(self.actual_position_y*1000).to_bytes( 4 , byteorder='little' , signed=True )
        actual_linear_velocity_x_bytes = int(self.actual_linear_velocity_x*1000).to_bytes( 4 , byteorder='little' , signed=True )
        actual_linear_velocity_y_bytes = int(self.actual_linear_velocity_y*1000).to_bytes( 4 , byteorder='little' , signed=True )
        actual_angular_velocity_z_bytes = int(self.actual_angular_velocity_z*1000).to_bytes( 4 , byteorder='little' , signed=True )
        actual_angle_bytes = int(self.actual_angle*1000).to_bytes( 4 , byteorder='little' , signed=True )
        message = [101] + list(actual_position_x_bytes) + list(actual_position_y_bytes) + list(actual_angle_bytes) + list(actual_linear_velocity_x_bytes) + list(actual_linear_velocity_y_bytes) + list(actual_angular_velocity_z_bytes)
        self.conn.sendall((bytes(message)))
        
        

        
    def tcp_loop(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = '10.0.100.145'
        port = 8899
        self.s.bind((host, port))
        self.connect()
        while True:
            try:
                data = self.conn.recv(1024)
                if not data:
                    self.conn_state = False
                    print("Client disconnected")
                    self.conn.close()
                    self.connect()
                    continue
                    
                    
                
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
                    #1 - jed na pozici
                    case 1:
                        print("GOAL POSITION")
                        x = float(int.from_bytes(data[1:5], byteorder='little', signed=True))/1000.0
                        y = float(int.from_bytes(data[5:9], byteorder='little', signed=True))/1000.0 
                        z = float(int.from_bytes(data[9:13], byteorder='little', signed=True))/1000.0                  
                        self.set_goal_position(x, y, z)
                        #state = States.POSITIONING
                    #2 - jed urcitou rychlosti
                    case 2:
                        print("VELOCITY")
                        x = float(int.from_bytes(data[1:5], byteorder='little', signed=True))/100.0
                        y = float(int.from_bytes(data[5:9], byteorder='little', signed=True))/100.0 
                        z = float(int.from_bytes(data[9:13], byteorder='little', signed=True))/100.0 
                        self.set_velocity(x,y, z)
                        #state = States.VEL_CONTROL

                    #4 - zastav vozitko
                    case 4:
                        print("STOP")
                        self.stop_rover()
                        #state = States.STOP
                    #5 - startovaci pozice    
                    case 5:
                        print("START POSITION")
                        x = float(int.from_bytes(data[1:5], byteorder='little', signed=True))/1000.0
                        y = float(int.from_bytes(data[5:9], byteorder='little', signed=True))/1000.0 
                        z = float(int.from_bytes(data[9:13], byteorder='little', signed=True))/1000.0 
                        self.set_initial_pose(x, y, z)
                    #11 - zapni mapovaní 
                    case 11:
                        print("START MAPPING")
                        # TODO: implement mapping start
                    #12 - vypni mapovaní
                    case 12:
                        print("STOP MAPPING")
                        # TODO: implement mapping stop
                    #13 - start navigace
                    case 13:
                        print("START NAVIGATION")
                        # TODO: implement navigation start
                    #14 - staop navigace
                    case 14: 
                        print("STOP NAVIGATION")
                        # TODO: implement navigation stop



                    case _:
                        pass
            except Exception as e:
                print(f"Exception: {e}")
                self.conn_state = False
                print("Client disconnected")
                self.conn.close()
                self.connect()
                continue

    def connect(self):
        self.s.listen()
        self.conn, addr = self.s.accept()
        print(f"Connected by {addr}")
        self.conn_state = True

    def set_initial_pose(self, x, y, angle):
        msg = PoseWithCovarianceStamped()
        header = Header()   
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map" 
        msg.header = header
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = float(angle)
        print(f"Initial pos: {x} {y} {angle}")
        self.publisher_initial_pose.publish(msg)

    def set_goal_position(self, x, y, angle):
        msg = PoseStamped()
        header = Header()   
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map" 
        msg.header = header
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = float(angle)
        print(f"Goal pos: {x} {y} {angle}")
        self.publisher_pose.publish(msg)
        
    def set_velocity(self, x, y, z):
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.linear.z = float(0)
        msg.angular.x = float(0)
        msg.angular.y = float(0)
        msg.angular.z = float(z)
        print(f"Velocity: {x} {y} {z}")
        self.publisher_twist.publish(msg)

    def stop_rover(self):
        msg = Twist()
        msg.linear.x = float(0)
        msg.linear.y = float(0)
        msg.linear.z = float(0)
        msg.angular.x = float(0)
        msg.angular.y = float(0)
        msg.angular.z = float(0)
        print(f"Velocity: {x} {y} {z}")
        self.publisher_twist.publish(msg)
        



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
