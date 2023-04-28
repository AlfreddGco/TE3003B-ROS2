import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import numpy as np
from time import time
from copy import copy

import pydash as _

from geometry_msgs.msg import Twist, Vector3
from geometry_msgs.msg import Transform, TransformStamped 
from tf2_geometry_msgs import PoseStamped
from tf2_msgs.msg import TFMessage

TOPIC_CALCULATED_POSE = '/calculated_pose'

def find_transform(transforms, child_frame_id):
    child_frames_aux = _.collections.map_(transforms, 'child_frame_id')
    return _.arrays.index_of(child_frames_aux, child_frame_id)


def sum_transforms(transform_a, transform_b):
    translation = copy(transform_a.translation)
    translation.x += transform_b.translation.x
    translation.y += transform_b.translation.y
    translation.z += transform_b.translation.z
    rotation = copy(transform_a.rotation)
    # quat -> euler -> sum(rot1, rot2) -> quat (?)
    #rotation.x = transform_b.rotation.x
    #rotation.y = transform_b.rotation.y
    #rotation.z = transform_b.rotation.z
    #rotation.w = transform_b.rotation.w
    transform = Transform()
    transform.translation = translation
    transform.rotation = rotation
    return transform


TOPIC_CALCULATED_POSE = '/calculated_pose'

class PuzzlebotTransformations(Node):
    def __init__(self):
        super().__init__('puzzlebot_tf')
        self.chassis_transform = TransformStamped()
        self.chassis_transform.header.frame_id = 'base_link'
        self.chassis_transform.child_frame_id = 'chassis'

        self.create_subscription(PoseStamped, TOPIC_CALCULATED_POSE,
            self.calculate_chassis_transform, 10)
        
        # robot_state_publisher transforms
        self.rs_transforms = []
        self.rs_static_transforms = []
        self.create_subscription(
            TFMessage, '/robot_state_publisher/tf',
            self.set_rs_transforms, 10)
        self.create_subscription(
            TFMessage, '/robot_state_publisher/tf_static',
            self.set_rs_static_transforms, 10)
        self.publisher = self.create_publisher(TFMessage, '/tf', 10)
        self.static_publisher = self.create_publisher(TFMessage, '/tf_static', 10)
        self.create_timer(1/20, self.run)


    def set_rs_transforms(self, msg):
        self.rs_transforms = msg.transforms


    def set_rs_static_transforms(self, msg):
        self.rs_static_transforms = msg.transforms


    def calculate_chassis_transform(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.chassis_transform.transform.translation.x = x
        self.chassis_transform.transform.translation.y = y
        self.chassis_transform.transform.rotation = msg.pose.orientation
        self.chassis_transform.header.stamp = self.get_clock().now().to_msg()


    def publish_transforms(self):
        msg = TFMessage()
        msg.transforms = [self.chassis_transform] + self.rs_transforms

        chassis_idx = find_transform(msg.transforms, 'chassis')
        left_idx = find_transform(msg.transforms, 'left_wheel')
        right_idx = find_transform(msg.transforms, 'right_wheel')
        if(chassis_idx == -1 or left_idx == -1 or right_idx == -1):
            self.publisher.publish(msg)
            return

        # Calculate wheels to base from wheels to chassis
        chassis_to_base = msg.transforms[chassis_idx]
        left_wheel_to_chassis = msg.transforms[left_idx]
        right_wheel_to_chassis = msg.transforms[right_idx]

        left_to_base = TransformStamped()
        left_to_base.header.stamp = self.get_clock().now().to_msg()
        left_to_base.header.frame_id = 'base_link'
        left_to_base.child_frame_id = 'left_wheel'
        left_to_base.transform = sum_transforms(
            left_wheel_to_chassis.transform, chassis_to_base.transform)

        right_to_base = TransformStamped()
        right_to_base.header.stamp = self.get_clock().now().to_msg()
        right_to_base.header.frame_id = 'base_link'
        right_to_base.child_frame_id = 'right_wheel'
        right_to_base.transform = sum_transforms(
            right_wheel_to_chassis.transform, chassis_to_base.transform)

        msg.transforms += [left_to_base, right_to_base]

        camera_to_chassis_idx = find_transform(self.rs_static_transforms, 'camera')
        if(camera_to_chassis_idx != -1):
            camera_to_chassis = self.rs_static_transforms[camera_to_chassis_idx]
            camera_to_base = TransformStamped()
            camera_to_base.header.stamp = self.get_clock().now().to_msg()
            camera_to_base.header.frame_id = 'base_link'
            camera_to_base.child_frame_id = 'camera'
            camera_to_base.transform = sum_transforms(
                camera_to_chassis.transform, chassis_to_base.transform)
            msg.transforms += [camera_to_base]

        self.publisher.publish(msg)


    def publish_static_transforms(self):
        # delete chassis from static (it is moving with respect to base)
        msg = TFMessage()
        msg.transforms = copy(self.rs_static_transforms)
        chassis_idx = find_transform(msg.transforms, 'chassis')
        if(chassis_idx != -1):
            del msg.transforms[chassis_idx]
            self.static_publisher.publish(msg)

    
    def run(self):
        self.publish_transforms()
        self.publish_static_transforms()


def main():
    rclpy.init()
    node = PuzzlebotTransformations()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
