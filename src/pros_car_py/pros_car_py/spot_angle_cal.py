
import numpy as np
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix
from math import *
# import matplotlib.pyplot as plt
from pros_car_py.pros_car_py.spot_util import RotMatrix3D, point_to_rad
import threading
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node
import rclpy
import orjson
import math

class kinematics(Node):
    
    def __init__(self):
        super().__init__('kinematics')
        # note: leg IDs
        self.LB_target = []
        self.RB_target = []
        self.LF_target = []
        self.RF_target = []

        self.phi = radians(90)
        
        # body dimensions
        self.length = 0.57058
        self.width = 0.21232
        self.hight = 0.0
        
        self.return_angles = []
        self.center = [0, 0, 0]
        self.xyz = [0, 0, -0.32]
        # leg origins (left_f, left_b, right_b, right_f), i.e., the coordinate of j1
        
        self.spot_subscriber = self.create_subscription(
            Float32MultiArray, 
            "LB_target", 
            self.recieve_LB_target, 
            1
        )

        self.spot_subscriber = self.create_subscription(
            Float32MultiArray, 
            "RB_target", 
            self.recieve_RB_target, 
            1
        )

        self.spot_subscriber = self.create_subscription(
            Float32MultiArray, 
            "LF_target", 
            self.recieve_LF_target, 
            1
        )

        self.spot_subscriber = self.create_subscription(
            Float32MultiArray, 
            "RF_target", 
            self.recieve_RF_target, 
            1
        )

        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint,
            'spot_joint_trajectory_point',
            10
        )
        self.timer_ = self.create_timer(0.2, self.timer_callback) #接到spotRosBridgeSubscriber 12joints 關節資料

    def angle_between_vectors(self, v1, v2): # 餘弦定理 得知馬達角度
    
        dot_product = np.dot(v1, v2)
        norm_v1 = np.linalg.norm(v1)
        norm_v2 = np.linalg.norm(v2)

        cos_angle = dot_product / (norm_v1 * norm_v2)
        
        # 由於浮點數運算的精度問題，cos_angle 可能會超出 [-1, 1] 的範圍
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        
        # range [0, pi]
        angle_radians = np.arccos(cos_angle)
        angle_deg = np.degrees(angle_radians)
        return angle_deg

    # def received_data(self.msg)
    #     self.get_logger().info('Received : %s' % str(msg.data))

    #     if msg.data:
    #         recieved_data = msg.data
    #         generated_angles = []
    #         for i in range(3):
    #             return_angles_per_leg = self.leg_IK(self.xyz, legID = i, center_offset = self.center)
    #             return_angles_per_leg = return_angles_per_leg[0:3]
    #             generated_angles.append(return_angles_per_leg)
    #             degree_values = [degrees(rad) for rad in return_angles_per_leg]
    #             print(f"angle for leg {i} is :  {degree_values}")

    #         """ generated : [0]:LF [1]:LB [2]:RF [3]:RB """    
    #         """ target    : [0]:LF [1]:RF [2]:RB [3]:LB """
    #         """ LF,LB: origin (-20+,20-) RF,RB: origin (20-,-20+)"""
    #         """ RF,RB's third link direction different from origin, need to inverse. """
    #         # self.joint_pos = [math.radians(0), math.radians(25), math.radians(-60), math.radians(0), math.radians(-25), math.radians(60),
    #         #                   math.radians(0), math.radians(-25), math.radians(60), math.radians(0), math.radians(25), math.radians(-60)]
    #         self.return_angles = []
    #         self.flatten_angles(generated_angles[0], 0)
    #         self.flatten_angles(generated_angles[2], 2)
    #         self.flatten_angles(generated_angles[3], 3)
    #         self.flatten_angles(generated_angles[1], 1)
    #         # xyz = recieved_data[0:3]
    #         # xyz = np.array(xyz)
    #         # origin = recieved_data[3:6]
    #         # origin = np.array(origin)
    #         # origin = origin + self.leg_origins[1]
    #         # xyz = xyz - origin
    #         # xyz = xyz.tolist()
    #         # self.return_angles = self.leg_IK(xyz)

    #         pub = JointTrajectoryPoint()
    #         # # msg.positions = [math.degrees(pos) for pos in self.joint_pos]   # Replace with actual desired positions
    #         pub.positions = [float(pos) for pos in self.return_angles]
    #         pub.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]  # Replace with actual desired velocities
    #         self.get_logger().info('return angles: %s' % str(pub.positions))
    #         self.joint_trajectory_publisher_.publish(pub)

    def recieve_LB_target(self, msg):
        # self.get_logger().info('Received : %s' % str(msg.data))
        if msg.data:
            self.LB_target = []
            tmp = []
            recieved_data = msg.data
            for i in range(len(recieved_data) // 3):
                tmp = []
                tmp.append(recieved_data[3*i])
                tmp.append(recieved_data[3*i + 1])
                tmp.append(recieved_data[3*i + 2])

                self.LB_target.append(tmp)
            # self.get_logger().info('LB : %s' % str(msg.data))
            # self.get_logger().info('LB : %s' % str(self.LB_target))
    
    def recieve_RB_target(self, msg):
        # self.get_logger().info('Received : %s' % str(msg.data))
        if msg.data:
            self.RB_target = []
            tmp = []
            recieved_data = msg.data
            for i in range(len(recieved_data) // 3):
                tmp = []
                tmp.append(recieved_data[3*i])
                tmp.append(recieved_data[3*i + 1])
                tmp.append(recieved_data[3*i + 2])

                self.RB_target.append(tmp)
            # self.get_logger().info('RB : %s' % str(msg.data))

    def recieve_LF_target(self, msg):
        # self.get_logger().info('Received : %s' % str(msg.data))
        if msg.data:
            self.LF_target = []
            tmp = []
            recieved_data = msg.data
            for i in range(len(recieved_data) // 3):
                tmp = []
                tmp.append(recieved_data[3*i])
                tmp.append(recieved_data[3*i + 1])
                tmp.append(recieved_data[3*i + 2])

                self.LF_target.append(tmp)
            # self.get_logger().info('LF : %s' % str(msg.data))

    def recieve_RF_target(self, msg):
        # self.get_logger().info('Received : %s' % str(msg.data))
        if msg.data:
            self.RF_target = []
            tmp = []
            recieved_data = msg.data
            for i in range(len(recieved_data) // 3):
                tmp = []
                tmp.append(recieved_data[3*i])
                tmp.append(recieved_data[3*i + 1])
                tmp.append(recieved_data[3*i + 2])

                self.RF_target.append(tmp)
            # self.get_logger().info('RF : %s' % str(msg.data))

    def timer_callback(self): 
        if(not self.LB_target or not self.RB_target or not self.LF_target or not self.RF_target): #若target資料未收齊 就不作動
            return
        
        self.return_angles = []
    # --- for LB leg: ---
        p1 = np.array(self.LB_target[3]) # shoulder Abduction direction joint position
        p2 = np.array(self.RB_target[3]) 
        p3 = np.array(self.LB_target[2]) # shoulder extension direction joint position
        p4 = np.array(self.LB_target[1]) # elbow extension direction joint position
        vec1 = p1 - p2  # body horizontal vector
        vec2 = p4 - p3 
        print(f"p1: {p1}")
        print(f"p2: {p2}")
        print(f"p3: {p3}")
        print(f"p4: {p4}")
        angle_LB_shoulder = self.angle_between_vectors(vec1, vec2) # 算shoulder Abduction角度

        p1 = np.array(self.LB_target[3])
        p2 = np.array(self.LF_target[3])
        p3 = np.array(self.LB_target[2])
        p4 = np.array(self.LB_target[1])
        vec1 = p1 - p2
        vec2 = p4 - p3
        angle_LB_thigh = self.angle_between_vectors(vec1, vec2)# 算shoulder extension角度

        p1 = np.array(self.LB_target[0])
        p2 = np.array(self.LB_target[1])
        p3 = np.array(self.LB_target[2])
        vec1 = p1 - p2
        vec2 = p3 - p2
        angle_LB_calf = self.angle_between_vectors(vec1, vec2) #算elbow extension角度
    # --- for RB leg: ---
        p1 = np.array(self.RB_target[3])
        p2 = np.array(self.LB_target[3])
        p3 = np.array(self.RB_target[2])
        p4 = np.array(self.RB_target[1])
        vec1 = p1 - p2
        vec2 = p4 - p3
        angle_RB_shoulder = self.angle_between_vectors(vec1, vec2)

        p1 = np.array(self.RB_target[3])
        p2 = np.array(self.RF_target[3])
        p3 = np.array(self.RB_target[2])
        p4 = np.array(self.RB_target[1])
        vec1 = p1 - p2
        vec2 = p4 - p3
        angle_RB_thigh = self.angle_between_vectors(vec1, vec2)

        p1 = np.array(self.RB_target[0])
        p2 = np.array(self.RB_target[1])
        p3 = np.array(self.RB_target[2])
        vec1 = p1 - p2
        vec2 = p3 - p2
        angle_RB_calf = self.angle_between_vectors(vec1, vec2)
    # --- for LF leg: ---
        p1 = np.array(self.LF_target[3])
        p2 = np.array(self.RF_target[3])
        p3 = np.array(self.LF_target[2])
        p4 = np.array(self.LF_target[1])
        vec1 = p1 - p2
        vec2 = p4 - p3
        angle_LF_shoulder = self.angle_between_vectors(vec1, vec2)

        p1 = np.array(self.LB_target[3])
        p2 = np.array(self.LF_target[3])
        p3 = np.array(self.LF_target[2])
        p4 = np.array(self.LF_target[1])
        vec1 = p1 - p2
        vec2 = p4 - p3
        angle_LF_thigh = self.angle_between_vectors(vec1, vec2)

        p1 = np.array(self.LF_target[0])
        p2 = np.array(self.LF_target[1])
        p3 = np.array(self.LF_target[2])
        vec1 = p1 - p2
        vec2 = p3 - p2
        angle_LF_calf = self.angle_between_vectors(vec1, vec2)
    # --- for RF leg: ---
        p1 = np.array(self.RF_target[3])
        p2 = np.array(self.LF_target[3])
        p3 = np.array(self.RF_target[2])
        p4 = np.array(self.RF_target[1])
        vec1 = p1 - p2
        vec2 = p4 - p3
        angle_RF_shoulder = self.angle_between_vectors(vec1, vec2)

        p1 = np.array(self.RB_target[3])
        p2 = np.array(self.RF_target[3])
        p3 = np.array(self.RF_target[2])
        p4 = np.array(self.RF_target[1])
        vec1 = p1 - p2
        vec2 = p4 - p3
        angle_RF_thigh = self.angle_between_vectors(vec1, vec2)

        p1 = np.array(self.RF_target[0])
        p2 = np.array(self.RF_target[1])
        p3 = np.array(self.RF_target[2])
        vec1 = p1 - p2
        vec2 = p3 - p2
        angle_RF_calf = self.angle_between_vectors(vec1, vec2)
    
    # --- send to unity ---
        """ target    : [0]:LF [1]:RF [2]:RB [3]:LB """
        self.return_angles.append(90 - angle_LF_shoulder)
        self.return_angles.append((-20) + 90 - angle_LF_thigh)
        self.return_angles.append(20 - 180 + angle_LF_calf)

        self.return_angles.append(angle_RF_shoulder - 90)
        self.return_angles.append(20 - 90 + angle_RF_thigh)
        self.return_angles.append((-20) + 180 - angle_RF_calf)

        self.return_angles.append(90 - angle_RB_shoulder)
        self.return_angles.append(20 - 90 + angle_RB_thigh)
        self.return_angles.append((-20) + 180 - angle_RB_calf)

        self.return_angles.append(angle_LB_shoulder - 90)
        self.return_angles.append((-20) + 90 - angle_LB_thigh)
        self.return_angles.append(20 - 180 + angle_LB_calf)

        self.LB_target = [] # to ensure there is another data in next time. 先清空資料 等下次再收到資料，且剛好是0.2秒再作動(line190)

        print(f"return_angle: {self.return_angles}")
        pub = JointTrajectoryPoint()
        return_angle_rad = [angle * math.pi / 180 for angle in self.return_angles]  #unity 收弧度
        pub.positions = [float(pos) for pos in return_angle_rad]

        # self.get_logger().info('return angles: %s' % str(pub.positions))
        self.joint_trajectory_publisher_.publish(pub)
        
        """ generated : [0]:LF [1]:LB [2]:RF [3]:RB """    
        """ target    : [0]:LF [1]:RF [2]:RB [3]:LB """
        """ LF,LB: origin (-20+,20-) RF,RB: origin (20-,-20+)"""
        """ RF,RB's third link direction different from origin, need to inverse. """

        # check = []
        # check.append(angle_LF_shoulder)
        # check.append(angle_LF_thigh)
        # check.append(angle_LF_calf)
        # check.append(angle_RF_shoulder)
        # check.append(angle_RF_thigh)
        # check.append(angle_RF_calf)
        # check.append(angle_RB_shoulder)
        # check.append(angle_RB_thigh)
        # check.append(angle_RB_calf)
        # check.append(angle_LB_shoulder)
        # check.append(angle_LB_thigh)
        # check.append(angle_LB_calf)
        # print(check)


def main(args=None):
    rclpy.init(args=args)
    node = kinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


