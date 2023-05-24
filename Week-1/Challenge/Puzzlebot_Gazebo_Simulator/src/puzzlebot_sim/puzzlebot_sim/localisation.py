#!/usr/bin/env python3
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import numpy as np
import math
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler 

class Localisation(Node):
    def __init__(self):
        super().__init__('puzzlebot_localisation')
        self.get_logger().info('localisation init...')
    
        self.rate = 1.0 / 100.0
        self.rate_creation = self.create_rate(100)

        self.wl, self.wr = .0 , .0
        self.kr, self.kl = 100.0, 100.0
        self.l, self.r = .08, .05
        self.pose = np.empty((3,1))
        self.sigmak = np.empty((3,3))
        self.covariance_matrix = np.array([0.0]*36)
        self.create_subscription(Float32, '/wl', self.update_wl, 10)
        self.create_subscription(Float32, '/wr', self.update_wr, 10)
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.odomConstants = self.create_odomerty_msg()

    
    def update_wl(self, wl: Float32):
        self.wl = wl.data

    def update_wr(self, wr: Float32):
        self.wr = wr.data

    def get_hk(self) -> np.ndarray:
        angle = self.pose[2].item()
        val1 = -self.v * math.sin(angle) / self.rate
        val2 = self.v * math.cos(angle) / self.rate
        return np.array([[1, 0, val1],
                         [0, 1, val2],
                         [0, 0, 1]])

    def get_qk(self) -> np.ndarray:
        angle = self.pose[2].item()
        dwk = (self.r/(2*self.rate))*np.array([[math.cos(angle), math.cos(angle)],
                       [math.sin(angle), math.sin(angle)],
                       [2/self.l, -2/self.l]])
        sdk = np.array([[self.kr*abs(self.wr), 0],
                        [0, self.kl*abs(self.wl)]])
        qk = np.dot(np.dot(dwk,sdk),dwk.T)
        return qk

    def get_point_msg(self, x_pose, y_pose, z_pose):
        point_msg = Point()
        point_msg.x = x_pose
        point_msg.y = y_pose
        point_msg.z = z_pose
        return point_msg

    def calculcate_covariance(self):
        hk = self.get_hk()
        Qk =  self.get_qk()
        self.sigmak = np.dot(np.dot(hk, self.sigmak), hk.T) + Qk
        self.odomConstants.pose.covariance[0] = self.sigmak[0][0] 
        self.odomConstants.pose.covariance[1] = self.sigmak[0][1]
        self.odomConstants.pose.covariance[5] = self.sigmak[0][2]
        self.odomConstants.pose.covariance[6] = self.sigmak[1][0]
        self.odomConstants.pose.covariance[7] = self.sigmak[1][1]
        self.odomConstants.pose.covariance[11] = self.sigmak[1][2]
        self.odomConstants.pose.covariance[30] = self.sigmak[2][0]
        self.odomConstants.pose.covariance[31] = self.sigmak[2][1]
        self.odomConstants.pose.covariance[35] = self.sigmak[2][2]
        
    def create_odomerty_msg(self)-> Odometry:
        odometry = Odometry()
        odometry.header.stamp = self.get_clock().now().to_msg()
        odometry.header.frame_id = 'base_link'
        odometry.child_frame_id = 'chassis'
        odometry.pose.pose.position.x = 0.0
        odometry.pose.pose.position.y = 0.0
        odometry.pose.pose.position.z = (self.r)
        odometry.pose.pose.orientation.x = 0.0
        odometry.pose.pose.orientation.y = 0.0
        odometry.pose.pose.orientation.z = 0.0
        odometry.pose.pose.orientation.w = 0.0
        odometry.pose.covariance = self.covariance_matrix
        
        odometry.twist.twist.linear.x = 0.0
        odometry.twist.twist.linear.y = 0.0
        odometry.twist.twist.linear.z = 0.0
        odometry.twist.twist.angular.x = 0.0
        odometry.twist.twist.angular.y = 0.0
        odometry.twist.twist.angular.z = 0.0
        odometry.twist.covariance = self.covariance_matrix
        return odometry
    
    def run(self):        
        while rclpy.ok():
            self.time_now = self.get_clock().now()
            
            self.v = self.r * (self.wr + self.wl) / 2 
            self.w = self.r * (self.wr - self.wl) / self.l

            angle = self.pose[2].item() 
            self.pose += np.array([[self.v * math.cos(angle)], 
                        [self.v * math.sin(angle)],
                        [self.w]])/self.rate 
            
            self.calculcate_covariance()

            self.odomConstants.pose.pose.position = self.get_point_msg(self.pose[0].item() , self.pose[1].item() , self.r)
            
            q = quaternion_from_euler(0, 0, self.pose[2].item())
            self.odomConstants.header.stamp = self.time_now.to_msg() #time stamp
            self.odomConstants.pose.pose.orientation.x = q[0]
            self.odomConstants.pose.pose.orientation.y = q[1]
            self.odomConstants.pose.pose.orientation.z = q[2]
            self.odomConstants.pose.pose.orientation.w = q[3]
            self.odomConstants.twist.twist.linear.x = self.v
            self.odomConstants.twist.twist.angular.z = self.w
            self.odom.publish(self.odomConstants)

            self.rate_creation.sleep()

def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = Localisation()
    executor.add_node(node)
    task = executor.create_task(node.run)
    try:
        executor.spin_until_future_complete(task)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if (__name__== '__main__') :
    main()
    
