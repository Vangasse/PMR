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

import sys
from cmath import inf
from dis import dis
from distutils.log import debug
import rclpy
import numpy as np
import math

from rclpy.node import Node

from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data



class Turtlebot3_Navigator(Node):

    def __init__(self):
        super().__init__('turtlebot3_navigator')
        qos = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_odom_ = self.create_publisher(Odometry, 'odom', 10)
        # self.scan_sub = self.create_subscription(
        #     LaserScan,
        #     '/scan',
        #     self.lidar_callback,
        #     qos_profile=qos_profile_sensor_data)
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos)

        self.objective = Point()
        goal = eval(sys.argv[1])
        print(goal)
        self.objective.x = float(goal[0])
        self.objective.y = float(goal[1])

        self.position = Point()
        self.orientation = Quaternion()
        self.Ts = .1
        self.k = .1
        self.d = .1

    def odom_callback(self, msg):
        self.update_pose(msg)
        roll, pitch, yaw = self.euler_from_quaternion(self.orientation)
        self.control(self.objective, self.position, yaw, self.d, self.k)

    def update_pose(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
 
    def euler_from_quaternion(self, orientation):
            """
            Convert a quaternion into euler angles (roll, pitch, yaw)
            roll is rotation around x in radians (counterclockwise)
            pitch is rotation around y in radians (counterclockwise)
            yaw is rotation around z in radians (counterclockwise)
            """
            x = orientation.x
            y = orientation.y
            z = orientation.z
            w = orientation.w

            t0 = +2.0 * (w * x + y * z)
            t1 = +1.0 - 2.0 * (x * x + y * y)
            roll_x = math.atan2(t0, t1)
        
            t2 = +2.0 * (w * y - z * x)
            t2 = +1.0 if t2 > +1.0 else t2
            t2 = -1.0 if t2 < -1.0 else t2
            pitch_y = math.asin(t2)
        
            t3 = +2.0 * (w * z + x * y)
            t4 = +1.0 - 2.0 * (y * y + z * z)
            yaw_z = math.atan2(t3, t4)
        
            return roll_x, pitch_y, yaw_z # in radians

    def control(self, objective, position, orientation, d, k):
        dx = k*(objective.x - position.x)
        dy = k*(objective.y - position.y)

        sin = np.sin(orientation)
        cos = np.cos(orientation)
        v = cos*dx + sin*dy
        w = (1/d)*(-sin*dx + cos*dy)

        cmd_vel_pub = Twist()

        cmd_vel_pub.linear.x = v
        cmd_vel_pub.angular.z = w
        
        self.publisher_.publish(cmd_vel_pub)

def main(args=None):
    rclpy.init(args=args)

    navigator = Turtlebot3_Navigator()

    rclpy.spin(navigator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
