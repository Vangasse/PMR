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

from ctypes import c_ulong
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



class Turtlebot3_TangentBug(Node):

    def __init__(self):
        super().__init__('turtlebot3_tangent_bug')
        qos = QoSProfile(depth=10)

        # Publishers
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

        # Subscribers
        self.sub_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile=qos_profile_sensor_data)

        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos)

        self.o2go = Point()
        self.goal = Point()

        goal_input = eval(sys.argv[1])
        self.goal.x = float(goal_input[0])
        self.goal.y = float(goal_input[1])

        self.position = Point()
        self.orientation = Quaternion()
        self.Ts = .1
        self.k = .3
        self.d = .1
        self.ranges = []
        self.angle_increment = 0
        self.range_max = 0

        self.epsilon = 0.5

        self.v = 0
        self.w = 0

        self.yaw = 0

    #################
    ### CALLBACKS ###
    #################

    def odom_callback(self, msg):
        self.update_pose(msg)
        roll, pitch, self.yaw = self.euler_from_quaternion(self.orientation)
        
    def lidar_callback(self, msg):
        self.ranges = msg.ranges
        self.angle_increment = msg.angle_increment 
        self.range_max = msg.range_max
        self.control()

    def update_pose(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
 
    #################
    ### FUNCTIONS ###
    #################

    def find_discontinuity_points(self):

        n_lasers = len(self.ranges)
        D_breaks = []
        o_breaks = []
        dist_before = -99999

        for o in range(n_lasers):
            
            dist = self.ranges[o]
            #print(dist_before - dist>30)
            if (dist_before - dist > 30):
                D_breaks.append(dist)
                o_breaks.append(o)

            dist_before = dist

        self.disc_points = []
        point = Point()

        for i in range(len(D_breaks)):
            ang = o_breaks[i]*self.angle_increment
            point.x = D_breaks[i]*np.cos(ang)
            point.y = D_breaks[i]*np.sin(ang)
            self.disc_points.append(point)


    def find_better_point(self):
        o_close = 0
        D_close = float("inf")

        for o in range(len(self.disc_points)):
            dist = np.sqrt((self.goal.x - self.disc_points[o].x)^2 + (self.goal.y - self.disc_points[o].y)^2 )
            if dist < D_close:
                o_close = o
                D_close = dist
        #print(o_close)
        self.o2go = self.disc_points[o_close] 

    
    #def follow_obstacle(self,yaw):

        #self.v = 
        #self.w = 

    def go2goal(self, objective):
        dx = self.k*(objective.x - self.position.x)
        dy = self.k*(objective.y - self.position.y)

        sin = np.sin(self.yaw)
        cos = np.cos(self.yaw)

        v = cos*dx + sin*dy
        w = (1/self.d)*(-sin*dx + cos*dy)

        w_max = 0.5
        if w > w_max:
            w = w_max
        elif w < - w_max:
            w = -w_max

        return v,w


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

    def control(self):
        self.infinity_ranges = all(np.array(self.ranges) > self.range_max ) #NA DIREÇÃO DO GOAL

        #print(self.ranges)
        #print(len(self.ranges)//8)
        #print(self.angle_increment) 
        #print(self.range_max)
        self.close2obstacle = not all(np.array(self.ranges[0:len(self.ranges)//8]+self.ranges[7*len(self.ranges)//8:]) > self.epsilon )

        print(self.infinity_ranges)
        print(self.close2obstacle)

        if self.infinity_ranges:
            v,w = self.go2goal(self.goal)
        else:
            self.find_discontinuity_points()
            if not self.close2obstacle:
                self.find_better_point()
                v,w = self.go2goal(self.o2go)
            else:
                #self.follow_obstacle()
                print('FOLLOW OBSTACLE')
                v = 0.0
                w = 0.0
                pass

        cmd_vel_pub = Twist()

        cmd_vel_pub.linear.x = v
        cmd_vel_pub.angular.z = w
        print(v,w)
        self.pub_cmd_vel.publish(cmd_vel_pub)

    #################
    ###### MAIN #####
    #################

def main(args=None):
    rclpy.init(args=args)

    navigator = Turtlebot3_TangentBug()

    rclpy.spin(navigator)

    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
