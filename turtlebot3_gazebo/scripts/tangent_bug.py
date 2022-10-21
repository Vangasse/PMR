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
from numpy.linalg import norm

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

        self.epsilon = 1
        self.v = 0
        self.w = 0
        self.phi_m = 0
        self.phi_m2 = 0
        self.phi = []

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

        #Compute the minimum distance and get the direction of the correspondent point
        self.delta_m = 1000000
        k_m = -1

        for k in range(len(self.ranges)):
            if(self.ranges[k] < self.delta_m and self.ranges[k] > 0.10):
                self.delta_m = self.ranges[k]
                k_m = k

        #Compute the associated direction in the body frame
        self.phi_m2 = k_m*self.angle_increment

        self.paralel = [self.delta_m*np.cos(self.phi_m2),self.delta_m*np.sin(self.phi_m2)]

        self.control()


    def update_pose(self, msg):
        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
 
    #################
    ### FUNCTIONS ###
    #################

    def find_direction(self):
        f1 = np.array([1,0])
        f2 = np.array([self.goal.x,self.goal.y])

        if np.cross(f1,f2) > 0:
            theta = np.arccos(f1.dot(f2)/(norm(f1)*norm(f2)))
        else:
            theta = -np.arccos(f1.dot(f2)/(norm(f1)*norm(f2)))
     
        try:
            tot = int((self.yaw - theta)*180/np.pi)
        except: 
            tot = 0
        
        if tot > 180:
            tot = tot - 360
        elif tot < -180:
            tot = 360 + tot

        front = np.hstack((self.ranges[-6:-1],self.ranges[0:6]))

        if abs(tot) < 6:
            self.direction = front
        elif tot < 0:
            self.direction = np.array(self.ranges[-tot-3:-tot+3])
        elif tot > 0:
            self.direction = np.array(self.ranges[-tot-3:-tot+3])

    def find_discontinuity_points(self):

        n_lasers = len(self.ranges)
        D_breaks = []
        o_breaks = []
        dist_before = self.ranges[0]

        for o in range(n_lasers):
            
            dist = self.ranges[o]
            if abs(dist_before - dist) > 0.1:
                if not (dist > 30):
                    D_breaks.append(dist)
                    o_breaks.append(o)
                else:

                    D_breaks.append(self.ranges[o-1])
                    o_breaks.append(o-1)

            dist_before = dist

        self.disc_points = []

        for i in range(len(D_breaks)):
            self.phi.append(o_breaks[i]*self.angle_increment)
            point_x = D_breaks[i]*np.cos(self.phi[i] + self.yaw) + self.position.x
            point_y = D_breaks[i]*np.sin(self.phi[i] + self.yaw) + self.position.y
            self.disc_points.append([point_x,point_y])

    def find_better_point(self):
        o_close = 0
        self.D_close = float("inf")
        
        for o in range(len(self.disc_points)):
            dist = np.sqrt((self.goal.x - self.disc_points[o][0])**2 + (self.goal.y - self.disc_points[o][1])**2 )
            dist2 = np.sqrt((self.position.x - self.disc_points[o][0])**2 + (self.position.y - self.disc_points[o][1])**2 )
            if dist + dist2 < self.D_close:
                o_close = o
                self.D_close = dist

        self.phi_m = self.phi[o_close]
        self.o2go.x = self.disc_points[o_close][0]
        self.o2go.y = self.disc_points[o_close][1] 

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

        return 0.5*v,0.5*w

    def follow_obstacle(self):
 
        hora = [[],[]]
        hora[0] = -np.sin(self.phi_m2 + self.yaw)
        hora[1] = np.cos(self.phi_m2 + self.yaw) 

        antihora = [[],[]]
        antihora[0] = np.sin(self.phi_m2 + self.yaw)
        antihora[1] = -np.cos(self.phi_m2 + self.yaw) 

        anghora = self.angle_of_vectors(hora[0],hora[1],self.o2go.x,self.o2go.y)
        angantihora = self.angle_of_vectors(antihora[0],antihora[1],self.o2go.x,self.o2go.y)

        G = (2/np.pi)*np.arctan(self.k*(self.delta_m - self.epsilon))
        H = np.sqrt(1-G*G)

        if anghora < angantihora:
            v = -(np.cos(self.phi_m2)*G - np.sin(self.phi_m2)*H)
            w = -(np.sin(self.phi_m2)*G/self.d + np.cos(self.phi_m2)*H/self.d)
        else:
            v = (np.cos(self.phi_m2)*G - np.sin(self.phi_m2)*H)
            w = (np.sin(self.phi_m2)*G/self.d + np.cos(self.phi_m2)*H/self.d)

        return v,0.1*w

    def angle_of_vectors(self,a,b,c,d):
        
        dotProduct = a*c + b*d
        modOfVector1 = np.sqrt( a*a + b*b)*np.sqrt(c*c + d*d) 
        
        angle = dotProduct/modOfVector1
        return angle

    def euler_from_quaternion(self, orientation):

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
        self.find_direction()

        self.infinity_ranges = all(self.direction > self.range_max )
        self.close2obstacle = not all(np.hstack((self.ranges[-100:-1],self.ranges[0:100])) > self.epsilon + 0.5)
        self.got_there = np.sqrt((self.goal.x - self.position.x)**2 + (self.goal.y - self.position.y)**2 ) < 0.8

        if not self.got_there:
            if self.infinity_ranges and not self.close2obstacle:
                v,w = self.go2goal(self.goal)
                Node.get_logger(self).info(f'Going do goal at {self.goal}', once=True)
            else:
                self.find_discontinuity_points()
                self.find_better_point()

                if not self.close2obstacle:
                    v,w = self.go2goal(self.o2go)
                    Node.get_logger(self).info(f'Going to discontinuity at {self.o2go}', once=False)
                else:
                    v,w = self.follow_obstacle()
                    
                    Node.get_logger(self).info('Following obstacle', once=True)
                    pass
        else:
            Node.get_logger(self).info('Simulation Finished', once=True)
            v = 0.0
            w = 0.0

        cmd_vel_pub = Twist()

        cmd_vel_pub.linear.x = v
        cmd_vel_pub.angular.z = w
        
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
