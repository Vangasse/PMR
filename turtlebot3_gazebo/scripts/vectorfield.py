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
from scipy.linalg import norm

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
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.publisher_odom_ = self.create_publisher(Odometry, 'odom', 10)

        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos)

        self.position = Point()
        self.pose = Point()
        self.orientation = Quaternion()

        # Cria curva
        theta = np.arange(0, 2*np.pi, 0.01)

        self.C = [[],[]]

        for theta1 in theta:
            self.C[0].append(2*np.cos(theta1)+0.2 * 2*np.sin(2*theta1)+0.05*np.sin(4*theta1) + 5.5)
            self.C[1].append(2*np.sin(theta1)-0.2 * 2*np.cos(2*theta1)-0.05*np.sin(4*theta1) + 5.5)

    ###################################################################################################
    #Callback da Pose toda vez que é publicada.
    def odom_callback(self, msg):
        self.update_pose(msg)
        roll, pitch, self.yaw = self.euler_from_quaternion(self.orientation)

        self.move_turtle()

    def update_pose(self, msg):
        self.position = msg.pose.pose.position
        self.pose.x = round(self.position.x, 4)
        self.pose.y = round(self.position.y, 4)
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



    ###################################################################################################
    # Encontra ponto mais próximo dentro do trajeto
    def pontomaisprox(self,dxatual,dyatual,C):
        d = [] # Distâncias
        
        for i in list(range(0,len(C[1]))):
            d.append(norm([self.pose.x + dxatual - C[0][i] , self.pose.y + dyatual - C[1][i]]))
        
        dmin = min(d) # Distância mínima
        j = d.index(min(d)) # Indíce correspondente à distância mínima
        
        p_star = [C[0][j],C[1][j]] # Ponto mais próximo
        
        norma = norm(np.array([p_star[0] - (self.pose.x + dxatual) ,p_star[1] - (self.pose.y + dyatual)])) # Norma da distância p p*
        
        N = [(p_star[0] - (self.pose.x + dxatual))/norma,(p_star[1] - (self.pose.y + dyatual))/norma] # Vetor normal
        
        if j == (len(C[1])-1):
            T = [C[0][0] - C[0][j] , C[1][0] - C[1][j]] # Caso especial
        else:
            T = [C[0][j+1] - C[0][j] , C[1][j+1] - C[1][j]] 

        T = np.array(T)/norm(T) #Vetor tangencial
        print(p_star[0]-self.pose.x)
        print(p_star[1]-self.pose.y)
        return [N,T,dmin]


    ###################################################################################################
    # Composição Vetor Normal e Tangente para Obter Velocidade Desejada no Estado
    def composicao_de_vetores (self,N,T,beta,dmin):
        
        G = (2/np.pi)*np.arctan(beta*dmin)
        H = np.sqrt(1-G**2)
        
        return 0.7*(G*np.array(N)+H*np.array(T))






    ###################################################################################################
    # Lei de Controle
    def control(self,C):
        
        pmed = np.array([self.pose.x + 0.1, self.pose.y + 0.1]) # Posição Medida (Simula Perturbações)

        deslocamento_atual = np.array([self.v*np.cos(self.yaw)*self.dt, self.v*np.sin(self.yaw)*self.dt])
        
        [N,T,dmin] = self.pontomaisprox(deslocamento_atual[0],deslocamento_atual[1],C)
        velocidade_desejada = self.composicao_de_vetores (N,T,self.beta,dmin)

        [N,T,dmin] = self.pontomaisprox(deslocamento_atual[0],deslocamento_atual[1],C)
        velocidade_desejada_futura = self.composicao_de_vetores (N,T,self.beta,dmin)

        derivada_velocidade = (velocidade_desejada_futura - velocidade_desejada)/self.dt

        w_max = 1
        k = 2 

        # Matriz
        M = np.array([[self.v*np.cos(self.yaw),self.v*np.sin(self.yaw)],
                      [-np.sin(self.yaw),np.cos(self.yaw)]])

        controlador_prop = -k*np.array([self.v*np.cos(self.yaw)-velocidade_desejada[0],self.v*np.sin(self.yaw)-velocidade_desejada[1]])

        acc_desired = [derivada_velocidade[0]+controlador_prop[0],derivada_velocidade[1]+controlador_prop[1]]


        [a,w] = np.matmul(M/self.v, acc_desired)
        
        if w > w_max:
            w = w_max
        elif w < -w_max:
            w = -w_max
        
        v = self.v + a*self.dt # Método de Euler
        


        #print(f"velocidade linear: {v}\nvelocidade angular: {w}\n")

        return [v,w]


    ###################################################################################################
    # Move Tartaruga Função Principal
    def move_turtle(self):
        vel = Twist()

        # Velocidades Iniciais
        self.v = 1
        self.w = 0.1

        # Parâmetros
        self.dt = 0.03
        self.beta = 3

        [self.v,self.w] = self.control(self.C)

        # Linear velocity in the x-axis.
        vel.linear.x = self.v
        vel.linear.y = 0.0
        vel.linear.z = 0.0

        # Angular velocity in the z-axis.
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = self.w

        # Publishing our vel_msg
        self.velocity_publisher.publish(vel)



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

