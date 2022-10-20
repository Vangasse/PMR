#!/usr/bin/env python3
from re import A
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Vector3
from nav_msgs.msg import Odometry
import math
import numpy as np
import sys

class ARP_ControllerNode(Node):
    
    def __init__(self):
        super().__init__("arp_controller_node")
        self.get_logger().info("ARP Controller Node Initialized")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.current_position = Point()
        self.current_orientation = Vector3()
        self.desired_position = Point()
        goal = eval(sys.argv[1])
        print("Goal: ", sys.argv[1], sys.argv[2])
        self.desired_position.x = float(eval(sys.argv[1]))
        self.desired_position.y = float(eval(sys.argv[2]))
        self.desired_position.z = 0.0        
        self.h = 0.1        
        self.obstacle_points = {}
        self.obstacle_distances = {}
        # Parâmetros de ajuste
        self.min_distance_to_the_obstacle = 1.0
        self.min_distance_to_the_goal = 5.0
        self.zeta = 1.0
        self.alpha = 0.1
        self.eta = 0.1
        self.epsilon = 0.5     
        self.run(int(eval(sys.argv[3])))
    
    def scan_callback(self, msg:LaserScan):

        # Obtendo os ângulos do laser
        laser_angles = np.fromfunction(lambda i: msg.angle_min+(i*msg.angle_increment), (len(msg.ranges),))
       
        # Dividindo os ângulos do laser por regiões
        right_angles = laser_angles[270:306]
        front_right_angles = laser_angles[306:342]
        front_angles = np.concatenate((laser_angles[0:18], laser_angles[342:]))
        front_left_angles = laser_angles[18:54]
        left_angles = laser_angles[54:90]

        # Dividindo as distâncias do laser por regiões
        right_distances = msg.ranges[270:306]
        front_right_distances = msg.ranges[306:342]
        front_distances = msg.ranges[0:18]+msg.ranges[342:]
        front_left_distances = msg.ranges[18:54]
        left_distances = msg.ranges[54:90]
     
        # Calculando
        self.obstacle_points = {}
        self.obstacle_distances = {}
        if(min(min(right_distances), 10) <= self.min_distance_to_the_obstacle):
            id = min(range(len(right_distances)), key=right_distances.__getitem__)
            self.obstacle_distances.update({"right": right_distances[id]})
            self.obstacle_points.update({"right": self.obstacle_point(right_distances[id], right_angles[id])})
            
        if(min(min(front_right_distances), 10) <= self.min_distance_to_the_obstacle):
            id = min(range(len(front_right_distances)), key=front_right_distances.__getitem__)
            self.obstacle_distances.update({"front_right": front_right_distances[id]})
            self.obstacle_points.update({"front_right": self.obstacle_point(front_right_distances[id], front_right_angles[id])})
            
        if(min(min(front_distances), 10)  <= self.min_distance_to_the_obstacle):
            id = min(range(len(front_distances)), key=front_distances.__getitem__)
            self.obstacle_distances.update({"front": front_distances[id]})
            self.obstacle_points.update({"front": self.obstacle_point(front_distances[id], front_angles[id])})
            
        if(min(min(front_left_distances), 10)  <= self.min_distance_to_the_obstacle):
            id = min(range(len(front_left_distances)), key=front_left_distances.__getitem__)
            self.obstacle_distances.update({"front_left": front_left_distances[id]})
            self.obstacle_points.update({"front_left": self.obstacle_point(front_left_distances[id], front_left_angles[id])})
            
        if(min(min(left_distances), 10) <= self.min_distance_to_the_obstacle):
            id = min(range(len(left_distances)), key=left_distances.__getitem__)
            self.obstacle_distances.update({"left": left_distances[id]})
            self.obstacle_points.update({"left": self.obstacle_point(left_distances[id], left_angles[id])})

    def obstacle_point(self, distance, angle):

        x = distance*np.cos(angle)  # Distância em x entre o obstáculo e o robô
        y = distance*np.sin(angle)  # Distância em y entre o obstáculo e o robô

        p_ro = np.array([x, y, 0, 1]) # Ponto do obstáculo em relação ao robô

        # Transformação Homogênea do robô para o inercial
        H_ir = np.array([
                    [np.cos(self.current_orientation.z), -np.sin(self.current_orientation.z), 0., self.current_position.x],
                    [np.sin(self.current_orientation.z),  np.cos(self.current_orientation.z), 0., self.current_position.y],
                    [0., 0., 1., 0.],
                    [0., 0., 0., 1.]
                   ])

        # Ponto do obstáculo em relação ao inercial  
        p_io = np.dot(H_ir, p_ro)  
        p = Point()
        p.x, p.y, p.z = p_io[0], p_io[1], p_io[2]        
        return p
    
    def odom_callback(self, msg:Odometry):
        # obtendo a posição x, y e z
        self.current_position = msg.pose.pose.position
        # obtendo row(x), pitch(y) e yaw(z)       
        self.current_orientation.x, self.current_orientation.y, self.current_orientation.z = self.euler_from_quaternion(msg.pose.pose.orientation)

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

    def distance_to_the_goal(self):
        return math.sqrt(pow(self.desired_position.x - self.current_position.x, 2) + pow(self.desired_position.y - self.current_position.y, 2))

    # def conic_function(self):
    #     return self.zeta * self.distance_to_the_goal()

    def distance_to_the_goal(self, qx, qy, qx_goal, qy_goal):
        return math.sqrt(pow(qx_goal- qx, 2) + pow(qy_goal - qy, 2))
    
    def conic_gradient(self, qx, qy, qx_goal, qy_goal):
        d = self.distance_to_the_goal(qx, qy, qx_goal, qy_goal)
        gx = (self.zeta/d)*(qx - qx_goal) 
        gy = (self.zeta/d)*(qy - qy_goal) 
        return gx, gy
    
    # def quadratic_function(self):
    #     return (1/2)*(self.zeta * pow(self.distance_to_the_goal(), 2))
    
    def quadratic_gradient(self, qx, qy, qx_goal, qy_goal):
        gx = self.zeta*(qx - qx_goal)
        gy = self.zeta*(qy - qy_goal)
        return gx, gy
    
    def conic_and_quadratic_gradient(self, qx, qy, qx_goal, qy_goal):
        d = self.distance_to_the_goal(qx, qy, qx_goal, qy_goal)
        gx = (self.min_distance_to_the_goal*self.zeta/d)*(qx - qx_goal)
        gy = (self.min_distance_to_the_goal*self.zeta/d)*(qy - qy_goal)
        return gx, gy
    
    def norm_gradient(self, x, y):
        return math.sqrt(pow(x, 2) + pow(y, 2))
    
    def run(self, option):

        if(option == 1):
            self.conic()
        elif(option == 2):
            self.quadratic()
        elif(option == 3):
            self.conic_and_quadratic()
        else:
            print("Error")
    
    def r_gradient(self, min_d, d, qi, ci):
        return self.eta*((1/min_d)-(1/d))*(1/pow(d,2))*((qi-ci)/d)
        
    def repulsive_gradient(self):

        gx = 0.0
        gy = 0.0
        if('right' in self.obstacle_points and 'right' in self.obstacle_distances):
            gx += self.r_gradient(self.min_distance_to_the_obstacle, self.obstacle_distances['right'], self.current_position.x, self.obstacle_points['right'].x)
            gy += self.r_gradient(self.min_distance_to_the_obstacle, self.obstacle_distances['right'], self.current_position.y, self.obstacle_points['right'].y) 
        if('front_right' in self.obstacle_points and 'front_right' in self.obstacle_distances):
            gx += self.r_gradient(self.min_distance_to_the_obstacle, self.obstacle_distances['front_right'], self.current_position.x, self.obstacle_points['front_right'].x)
            gy += self.r_gradient(self.min_distance_to_the_obstacle, self.obstacle_distances['front_right'], self.current_position.y, self.obstacle_points['front_right'].y)
        if('front' in self.obstacle_points and 'front' in self.obstacle_distances):
            gx += self.r_gradient(self.min_distance_to_the_obstacle, self.obstacle_distances['front'], self.current_position.x, self.obstacle_points['front'].x)
            gy += self.r_gradient(self.min_distance_to_the_obstacle, self.obstacle_distances['front'], self.current_position.y, self.obstacle_points['front'].y)            
        if('front_left' in self.obstacle_points and 'front_left' in self.obstacle_distances):
            gx += self.r_gradient(self.min_distance_to_the_obstacle, self.obstacle_distances['front_left'], self.current_position.x, self.obstacle_points['front_left'].x)
            gy += self.r_gradient(self.min_distance_to_the_obstacle, self.obstacle_distances['front_left'], self.current_position.y, self.obstacle_points['front_left'].y) 
        if('left' in self.obstacle_points and 'left' in self.obstacle_distances):
            gx += self.r_gradient(self.min_distance_to_the_obstacle, self.obstacle_distances['left'], self.current_position.x, self.obstacle_points['left'].x)
            gy += self.r_gradient(self.min_distance_to_the_obstacle, self.obstacle_distances['left'], self.current_position.y, self.obstacle_points['left'].y) 

        return gx, gy
        
    def conic(self):
        print("Conic Potential")
        self.zeta = 1.0
        self.alpha = 0.1
        self.eta = 1.0
       
        gx_att, gy_att = self.conic_gradient(self.current_position.x, self.current_position.y, self.desired_position.x, self.desired_position.y)
        gx_rep, gy_rep = self.repulsive_gradient()

        twist_msg = Twist()
        while not self.norm_gradient(gx_att, gy_att) < self.epsilon:       
            print('x: ', round(self.current_position.x, 5), 'y: ', round(self.current_position.y, 5))
   
            ux = -self.alpha * (gx_att + gx_rep)
            uy = -self.alpha * (gy_att + gy_rep)
     
            v = ux*math.cos(self.current_orientation.z) + uy*math.sin(self.current_orientation.z)
            w = (1/self.h)*(-ux*math.sin(self.current_orientation.z) + uy*math.cos(self.current_orientation.z))

            twist_msg.linear.x = v
            twist_msg.angular.z = w
            self.cmd_vel_pub.publish(twist_msg)

            gx_att, gy_att = self.conic_gradient(self.current_position.x, self.current_position.y, self.desired_position.x, self.desired_position.y)
            gx_rep, gy_rep = self.repulsive_gradient()
            rclpy.spin_once(self, timeout_sec=1.0)
        
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        print('Done 1!')


    def quadratic(self):
        print("Quadratic Potential")
        self.zeta = 1.0
        self.alpha = 0.1
        self.eta = 1.0
                
        gx_att, gy_att = self.quadratic_gradient(self.current_position.x, self.current_position.y, self.desired_position.x, self.desired_position.y)
        gx_rep, gy_rep = self.repulsive_gradient()

        twist_msg = Twist()
        while not self.norm_gradient(gx_att,gy_att) < self.epsilon:
            print('x: ', round(self.current_position.x, 5), 'y: ', round(self.current_position.y, 5))

            ux = self.alpha * (-gx_att + gx_rep)
            uy = self.alpha * (-gy_att + gy_rep)

            v = ux*math.cos(self.current_orientation.z) + uy*math.sin(self.current_orientation.z)
            w = (1/self.h)*(-ux*math.sin(self.current_orientation.z) + uy*math.cos(self.current_orientation.z))

            twist_msg.linear.x = v
            twist_msg.angular.z = w
            self.cmd_vel_pub.publish(twist_msg)

            gx_att, gy_att = self.quadratic_gradient(self.current_position.x, self.current_position.y, self.desired_position.x, self.desired_position.y)
            gx_rep, gy_rep = self.repulsive_gradient()

            rclpy.spin_once(self, timeout_sec=1.0)

        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        print('Done 2!')

    def conic_and_quadratic(self):
        print("Conic and Quadratic Potential")
        self.zeta = 1.0
        self.alpha = 0.1
        self.eta = 1.0
        self.min_distance_to_the_goal = 5.0
                
        gx_att, gy_att = self.conic_gradient(self.current_position.x, self.current_position.y, self.desired_position.x, self.desired_position.y)
        gx_rep, gy_rep = self.repulsive_gradient()

        twist_msg = Twist()
        while not self.norm_gradient(gx_att,gy_att) < self.epsilon:
            print('x: ', round(self.current_position.x, 5), 'y: ', round(self.current_position.y, 5))

            ux = self.alpha * (-gx_att + gx_rep)
            uy = self.alpha * (-gy_att + gy_rep)

            # ux = -self.alpha * (gx_att + gx_rep)
            # uy = -self.alpha * (gy_att + gy_rep)

            v = ux*math.cos(self.current_orientation.z) + uy*math.sin(self.current_orientation.z)
            w = (1/self.h)*(-ux*math.sin(self.current_orientation.z) + uy*math.cos(self.current_orientation.z))

            twist_msg.linear.x = v
            twist_msg.angular.z = w
            self.cmd_vel_pub.publish(twist_msg)

            if self.distance_to_the_goal(self.current_position.x, self.current_position.y, self.desired_position.x, self.desired_position.y) <= self.min_distance_to_the_goal:
                gx_att, gy_att = self.quadratic_gradient(self.current_position.x, self.current_position.y, self.desired_position.x, self.desired_position.y)
            else:
                gx_att, gy_att = self.conic_and_quadratic_gradient(self.current_position.x, self.current_position.y, self.desired_position.x, self.desired_position.y)
            gx_rep, gy_rep = self.repulsive_gradient()

            rclpy.spin_once(self, timeout_sec=1.0)

        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        print('Done 3!')



    

def main(args=None):

    rclpy.init(args=args)
    node = ARP_ControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()        

if __name__ == '__main__':
    main()


