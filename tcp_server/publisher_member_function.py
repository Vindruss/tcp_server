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

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry 

from std_msgs.msg import Header


from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):
    velocity = 0
    x = 0.0
    y = 0.0
    def __init__(self):
        # super().__init__('minimal_publisher')
        # #self.publisher_ = self.create_publisher(String, 'topic', 10)
        # self.publisher_ = self.create_publisher(Twist, 'cmd_vel_out', 10)
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0
        
        # t1 = threading.Thread(target=self.tcp_loop, args=())
        # t1.start()


        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.listener_callback,
            10)
        self.subscription  

    def listener_callback(self, msg):     
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info('Position: ' + str(x) + ' ' + str(y))
    def timer_callback(self):
        #msg = String()
        #msg.data = 'Hello World: %d' % self.i
        #self.publisher_.publish(msg)
        #self.get_logger().info('Publishing: "%s"' % msg.data)
        #self.i += 1
        
        outMsg = Twist()
        #outMsg.header = Header()
        outMsg.linear.x = self.velocity
        #outMsg.header.stamp = self.get_clock().now().to_msg()
        #outMsg.header.frame_id = self.frame_id
        #outMsg.twist = inMsg
        
        if self.velocity > 0:
            self.publisher_.publish(outMsg)
        
        #self.get_logger().info('Publishing: "%s"' % "cmd_vel_out sended)
        
    def tcp_loop(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = '192.168.0.1'
        port = 8899
        print(f"TCP socket sonn.rectarted")
        s.bind((host, port))
        s.listen()
        conn, addr = s.accept()
        print(f"Connected by {addr}")
        while True:
            data = conn.recv(1024)
            if not data:
                break
                
            
            if data[0] == 97:
                conn.send(b"spravne")
                #self.velocity = 1.0
            else: 
                conn.send(b"spatne")
                #self.velocity = 0.0



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
