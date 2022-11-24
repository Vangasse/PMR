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
import os

from rclpy.node import Node

from geometry_msgs.msg import Twist, Point, Quaternion
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, PoseStamped
from rclpy.qos import QoSProfile
from rclpy.qos import qos_profile_sensor_data



class Turtlebot3_Navigator(Node):

    def __init__(self,path_discrete,flag):
        super().__init__('turtlebot3_navigator')
        qos = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_odom_ = self.create_publisher(Odometry, 'odom', 10)
        self.publisher_trail = self.create_publisher(Path, 'trail', 10)
        self.publisher_objective = self.create_publisher(Path, 'objective', 10)

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos)

        self.position = Point()
        self.orientation = Quaternion()
        self.velocity = 0.0
        self.k = .1
        self.d = .1
        self.v_lim = 1.0
        self.w_lim = np.pi/6

        self.objective_curve = Path()
        self.objective_curve.header.frame_id = "odom"
        self.objective_curve.header.stamp = self.get_clock().now().to_msg()
        
        self.trail = Path()
        self.trail.header.frame_id = "odom"
        self.trail.header.stamp = self.get_clock().now().to_msg()

        self.flag_closed_path = flag
        self.end_of_path = False
        self.following_path = False
        self.end_of_simulation = False
        self.int_error = 0
        self.der_error = 0
        self.path = [[],[]]

        # Cria curva
        theta = np.arange(0, 2*np.pi, 0.01)

        self.path_discrete = path_discrete
        self.step = 0

        vector_1 = [1,0]
        vector_2 = [ self.path_discrete[self.step+1][0] - self.path_discrete[self.step][0] , self.path_discrete[self.step+1][1] - self.path_discrete[self.step][1] ]

        unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
        unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
        dot_product = np.dot(unit_vector_1, unit_vector_2)


        cross_product = np.cross(vector_1,vector_2)

        if cross_product > 0:
            self.yaw_ref = np.arccos(dot_product)
        else:
            self.yaw_ref = -np.arccos(dot_product)

        # for theta1 in theta:
            
        #     self.C[0].append(3*np.sin(theta1) + np.sin(10*theta1))
        #     self.C[1].append(3*np.cos(theta1) + np.cos(10*theta1))
        
        #     self.objective_step = PoseStamped()
        #     self.objective_step.header.frame_id = "odom"
        #     self.objective_step.header.stamp = self.get_clock().now().to_msg()

        #     self.objective_step.pose.position.x = self.C[0][-1]
        #     self.objective_step.pose.position.y = self.C[1][-1]

        #     self.objective_curve.poses.append(self.objective_step)

    ###################################################################################################
    #Callback da Pose toda vez que é publicada.
    def odom_callback(self, msg):
        self.update_state(msg)
        roll, pitch, self.yaw = self.euler_from_quaternion(self.orientation)

        self.control(self.position, self.velocity, self.yaw, self.d, self.k, .03, 3)

        self.publisher_trail.publish(self.trail)
        self.publisher_objective.publish(self.objective_curve)

    def update_state(self, msg):
        self.trail_step = PoseStamped()
        self.trail_step.header.frame_id = "odom"
        self.trail_step.header.stamp = self.get_clock().now().to_msg()

        self.trail_step.pose.position = self.position
        self.trail_step.pose.orientation = self.orientation
        self.trail.poses.append(self.trail_step)

        self.position = msg.pose.pose.position
        self.orientation = msg.pose.pose.orientation
        self.velocity = msg.twist.twist.linear.x
 
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


    ###################################################################################################
    # Find Continuous path between two nodes
    def discrete_to_continuos(self, nodes):
        path = [[],[]]
        for i in range(len(nodes) - 1):
            path[0] = list(np.hstack((path[0] , np.linspace(nodes[i][0],nodes[i+1][0],10))))
            path[1] = list(np.hstack((path[1] , np.linspace(nodes[i][1],nodes[i+1][1],10))))

        return path 

    ###################################################################################################
    # Encontra ponto mais próximo dentro do trajeto
    def closest_point(self, position, dx, dy, C):
        d = [] # Distâncias
        
        for i in list(range(0,len(C[1]))):
            d.append(np.linalg.norm([position.x + dx - C[0][i] , position.y + dy - C[1][i]]))
        
        dmin = min(d) # Distância mínima
        j = d.index(min(d)) # Indíce correspondente à distância mínima
        
        p_star = [C[0][j],C[1][j]] # Ponto mais próximo
        
        norma = np.linalg.norm(np.array([p_star[0] - (position.x + dx) ,p_star[1] - (position.y + dy)])) # Norma da distância p p*
        
        N = [(p_star[0] - (position.x + dx))/norma,(p_star[1] - (position.y + dy))/norma] # Vetor normal

        if not self.flag_closed_path:
            if j >= (len(C[1])-1):
                self.end_of_path = True
                if self.step == len(self.path_discrete) - 1:
                    self.end_of_simulation = True

        if j == (len(C[1])-1):
            T = [C[0][0] - C[0][j] , C[1][0] - C[1][j]] # Caso especial
        else:
            T = [C[0][j+1] - C[0][j] , C[1][j+1] - C[1][j]] 

        T = np.array(T)/np.linalg.norm(T)
        return [N,T,dmin]

    ###################################################################################################
    # Composição Vetor Normal e Tangente para Obter Velocidade Desejada no Estado
    def vector_composition(self, N, T, beta, dmin):
        
        G = (2/np.pi)*np.arctan(beta*dmin)
        H = np.sqrt(1-G**2)
        
        return 0.7*(G*np.array(N)+H*np.array(T))

    ###################################################################################################
    # Lei de Controle
    def control(self, position, velocity, orientation, d, k, dt, beta):

        cmd_vel_pub = Twist()

        if self.end_of_simulation:
            v = 0.0
            w = 0.0 
        
        elif self.following_path:
            self.int_error = 0
            self.der_error = 0

            current_dx = velocity*dt*np.cos(orientation)
            current_dy = velocity*dt*np.sin(orientation)
            
            [N, T, dmin] = self.closest_point(position, current_dx, current_dy, self.path)
            dx, dy = self.vector_composition(N, T, beta, dmin)

            sin = np.sin(orientation)
            cos = np.cos(orientation)
            v = cos*dx + sin*dy
            w = (1/d)*(-sin*dx + cos*dy)


            if self.end_of_path and self.following_path:
                self.following_path = False
                v = 0.0
                w = 0.0

                if self.step == len(self.path_discrete) - 1:
                    self.end_of_simulation = True
                    print("Simulation Finished")
                else:      
                    vector_1 = [1,0]
                    vector_2 = [ self.path_discrete[self.step+1][0] - self.path_discrete[self.step][0] , self.path_discrete[self.step+1][1] - self.path_discrete[self.step][1] ]

                    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
                    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
                    dot_product = np.dot(unit_vector_1, unit_vector_2)


                    cross_product = np.cross(vector_1,vector_2)
                    if cross_product > 0:
                        self.yaw_ref = np.arccos(dot_product)
                    else:
                        self.yaw_ref = -np.arccos(dot_product)



            if v < self.v_lim:
                cmd_vel_pub.linear.x = v
            else:
                cmd_vel_pub.linear.x = self.v_lim
            
            if w < self.w_lim:
                cmd_vel_pub.angular.z = w
            else:
                cmd_vel_pub.angular.z = self.w_lim
        

        else:

            cmd_vel_pub.linear.x = 0.0
            
            if self.yaw_ref*self.yaw < 0 and (abs(self.yaw) + abs(self.yaw_ref)):
                if self.yaw > 0:
                    error = 2*np.pi - (self.yaw_ref - self.yaw)
                else:
                    error = -(2*np.pi - (self.yaw_ref - self.yaw))
            else:
                error = (self.yaw_ref - self.yaw)

            self.int_error += error

            w = 0.4*error + 0.01*self.int_error - 10.0*(self.der_error - error)

            self.der_error = error

            if np.abs(self.yaw_ref - self.yaw) < 0.1:
                self.following_path = True
                self.end_of_path = False

                self.path = self.discrete_to_continuos([self.path_discrete[self.step],self.path_discrete[self.step+1]])

                self.step += 1


            if w < self.w_lim:
                cmd_vel_pub.angular.z = w
            else:
                cmd_vel_pub.angular.z = self.w_lim






        self.publisher_.publish(cmd_vel_pub)



























def main(args=None):


    flag_method = 1


    if flag_method == 1:
        path_discrete = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'path_a_star.npy'))
    elif flag_method == 2:
        path_discrete = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'path_PRM.npy'))
    elif flag_method == 3:
        path_discrete = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), '../img/path_RRT2.npy'))
    else:
        print("Invalid Method")    
    
    plath_is_closed = False
    
    rclpy.init(args=args)

    navigator = Turtlebot3_Navigator(path_discrete,plath_is_closed)

    rclpy.spin(navigator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

