
import numpy as np
from numpy.linalg import inv, norm
from numpy import array, asarray, matrix
from math import *
# import matplotlib.pyplot as plt
from pros_car_py.pros_car_py.spot_util import RotMatrix3D, point_to_rad
import threading
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import String
from rclpy.node import Node
import rclpy
import orjson
import math

class kinematics(Node):
    
    def __init__(self):
        super().__init__('kinematics')
        # note: leg IDs
        left_front = 0
        left_back  = 1
        right_front= 2
        right_back = 3
        
        self.right_legs = [right_front, right_back]
        
        self.link_1 = 0.039
        self.link_2 = 0.208
        self.link_3 = 0.208
        self.phi = radians(90)
        
        # body dimensions
        self.length = 0.57058
        self.width = 0.21232
        self.hight = 0.0
        
        self.return_angles = []
        self.center = [0, 0, 0]
        self.xyz = [0, 0, -0.32]
        # leg origins (left_f, left_b, right_b, right_f), i.e., the coordinate of j1
        self.leg_origins = np.matrix([[self.length/2, self.width/2, 0],
                          [-self.length/2, self.width/2, 0],
                          [-self.length/2, -self.width/2, 0],
                          [self.length/2, -self.width/2, 0]])
        
        self.spot_subscriber = self.create_subscription(
            String, 
            "center_dir", 
            self.recieve_center_movement, 
            1
        )

        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint,
            'spot_joint_trajectory_point',
            10
        )

    def recieve_center_movement(self, msg):
        self.get_logger().info('Received : %s' % str(msg.data))

        if msg.data:
            recieved_data = msg.data
            
            if (recieved_data == "left"):
                self.center[1] += 0.03
            elif (recieved_data == "right"):
                self.center[1] -= 0.03
            elif (recieved_data == "front"):
                self.center[0] += 0.03
            elif (recieved_data == "back"):
                self.center[0] -= 0.03
            elif (recieved_data == "origin"):
                self.center = [0, 0, 0]
            generated_angles = []
            for i in range(4):
                return_angles_per_leg = self.leg_IK(self.xyz, legID = i, center_offset = self.center)
                return_angles_per_leg = return_angles_per_leg[0:3]
                generated_angles.append(return_angles_per_leg)
                degree_values = [degrees(rad) for rad in return_angles_per_leg]
                print(f"angle for leg {i} is :  {degree_values}")

            """ generated : [0]:LF [1]:LB [2]:RF [3]:RB """    
            """ target    : [0]:LF [1]:RF [2]:RB [3]:LB """
            """ LF,LB: origin (-20+,20-) RF,RB: origin (20-,-20+)"""
            """ RF,RB's third link direction different from origin, need to inverse. """
            # self.joint_pos = [math.radians(0), math.radians(25), math.radians(-60), math.radians(0), math.radians(-25), math.radians(60),
            #                   math.radians(0), math.radians(-25), math.radians(60), math.radians(0), math.radians(25), math.radians(-60)]
            self.return_angles = []
            self.flatten_angles(generated_angles[0], 0)
            self.flatten_angles(generated_angles[2], 2)
            self.flatten_angles(generated_angles[3], 3)
            self.flatten_angles(generated_angles[1], 1)
            # xyz = recieved_data[0:3]
            # xyz = np.array(xyz)
            # origin = recieved_data[3:6]
            # origin = np.array(origin)
            # origin = origin + self.leg_origins[1]
            # xyz = xyz - origin
            # xyz = xyz.tolist()
            # self.return_angles = self.leg_IK(xyz)

            pub = JointTrajectoryPoint()
            # # msg.positions = [math.degrees(pos) for pos in self.joint_pos]   # Replace with actual desired positions
            pub.positions = [float(pos) for pos in self.return_angles]
            pub.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]  # Replace with actual desired velocities
            self.get_logger().info('return angles: %s' % str(pub.positions))
            self.joint_trajectory_publisher_.publish(pub)

    def flatten_angles(self, list, leg_ID):
        if (leg_ID == 0 or leg_ID == 1):
            list[1] += 45
            list[2] -= 45
        elif (leg_ID == 2 or leg_ID == 3):
            list[1] -= 45
            list[2] *= -1
            list[2] += 45
        for i in range(3):
            tmp = math.radians(list[i])
            self.return_angles.append(tmp)

    # this method adjust inputs to the IK calculator by adding rotation and 
    # offset of that rotation from the center of the robot
    def leg_IK(self, xyz, rot = [0,0,0], legID=0, is_radians=True, center_offset=[0,0,0]):
        
        # check is the leg is from the right side 
        is_right = (legID in self.right_legs)

        addition_list = np.array(center_offset)
        xyz -= addition_list

        # add offset of each leg from the axis of rotation
        XYZ = asarray((inv(RotMatrix3D(rot,is_radians)) * \
            ((array(xyz) + self.leg_origins[legID,:] - array(center_offset)).transpose())).transpose())
        
        # subtract the offset between the leg and the center of rotation 
        # so that the resultant coordiante is relative to the origin (j1) of the leg
        xyz_ = asarray(XYZ - self.leg_origins[legID,:] + array(center_offset)).flatten()

        # calculate the angles and coordinates of the leg relative to the origin of the leg
        return self.leg_IK_calc(xyz_, is_right)


    # IK calculator
    def leg_IK_calc(self, xyz, is_right=False): 

        x, y, z = xyz[0], xyz[1], xyz[2]    # unpack coordinates
        
        # length of vector projected on the YZ plane. equiv. to len_A = sqrt(y**2 + z**2)
        len_A = norm([0,y,z])   
        
        # a_1 : angle from the positive y-axis to the end-effector (0 <= a_1 < 2pi)
        # a_2 : angle bewtween len_A and leg's projection line on YZ plane
        # a_3 : angle between link1 and length len_A
        a_1 = point_to_rad(y,z)                     
        a_2 = asin(sin(self.phi)*self.link_1/len_A) 
        a_3 = pi - a_2 - self.phi                   
        
        # angle of link1 about the x-axis 
        if is_right: theta_1 = a_1 - a_3
        else: 
            theta_1 = a_1 + a_3
            if theta_1 >= 2*pi: theta_1 -= 2*pi
        
        j2 = array([0,self.link_1*cos(theta_1),self.link_1*sin(theta_1)])
        j4 = array(xyz)
        j4_2_vec = j4 - j2 # vector from j2 to j4
        
        if is_right: R = theta_1 - self.phi - pi/2
        else: R = theta_1 + self.phi - pi/2
        
        # create rotation matrix to work on a new 2D plane (XZ_)
        rot_mtx = RotMatrix3D([-R,0,0],is_radians=True)
        j4_2_vec_ = rot_mtx * (np.reshape(j4_2_vec,[3,1]))
        
        # xyz in the rotated coordinate system + offset due to link_1 removed
        x_, y_, z_ = j4_2_vec_[0], j4_2_vec_[1], j4_2_vec_[2]
        
        len_B = norm([x_, z_]) # norm(j4-j2)
        
        # handling mathematically invalid input, i.e., point too far away to reach
        if len_B >= (self.link_2 + self.link_3): 
            len_B = (self.link_2 + self.link_3) * 0.99999
            # self.node.get_logger().warn('target coordinate: [%f %f %f] too far away' % (x, y, z))
            print('target coordinate: [%f %f %f] too far away' % (x, y, z))
        
        # b_1 : angle between +ve x-axis and len_B (0 <= b_1 < 2pi)
        # b_2 : angle between len_B and link_2
        # b_3 : angle between link_2 and link_3
        b_1 = point_to_rad(x_, z_)  
        b_2 = acos((self.link_2**2 + len_B**2 - self.link_3**2) / (2 * self.link_2 * len_B)) 
        b_3 = acos((self.link_2**2 + self.link_3**2 - len_B**2) / (2 * self.link_2 * self.link_3))  
        
        # assuming theta_2 = 0 when the leg is pointing down (i.e., 270 degrees offset from the +ve x-axis)
        theta_2 = b_1 - b_2    
        theta_3 = pi - b_3
        
        # CALCULATE THE COORDINATES OF THE JOINTS FOR VISUALIZATION
        j1 = np.array([0,0,0])
        
        # calculate joint 3
        j3_ = np.reshape(np.array([self.link_2*cos(theta_2),0, self.link_2*sin(theta_2)]),[3,1])
        j3 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx)*j3_, [1,3])).flatten()
        
        # calculate joint 4
        j4_ = j3_ + np.reshape(np.array([self.link_3*cos(theta_2+theta_3),0, self.link_3*sin(theta_2+theta_3)]), [3,1])
        j4 = np.asarray(j2 + np.reshape(np.linalg.inv(rot_mtx)*j4_, [1,3])).flatten()
        
        # modify angles to match robot's configuration (i.e., adding offsets)
        angles = self.angle_corrector(angles=[theta_1, theta_2, theta_3], is_right=is_right)
        # print(degrees(angles[0]))
        # return [angles[0], angles[1], angles[2], j1, j2, j3, j4]
        return [angles[0], angles[1], angles[2]]
    
    
    # def base_pose(self, rot=[0,0,0], is_radians=True, center_offset=[0,0,0]):
        
    #     # offset due to non-centered axes of rotation
    #     offset = RotMatrix3D(rot, is_radians) * \
    #         (matrix(center_offset).transpose()) - matrix(center_offset).transpose()
        
    #     # rotate the base around the center of rotation (if there is no offset, then the center of 
    #     # rotation will be at the center of the robot)
    #     rotated_base = RotMatrix3D(rot, is_radians) * self.leg_origins.transpose() - offset
    #     return rotated_base.transpose()
       
    # get coordinates of leg joints relative to j1
    # def leg_pose(self, xyz, rot, legID, is_radians, center_offset=[0,0,0]):
        
    #     # get the coordinates of each joints relative to the leg's origin
    #     pose_relative = self.leg_IK(xyz, rot, legID, is_radians, center_offset)[3:]
        
    #     # adjust the coordinates according to the robot's orientation (roll, pitch, yaw)
    #     pose_true = RotMatrix3D(rot,is_radians) * (array(pose_relative).transpose())
    #     return pose_true.transpose()
    
    # plot rectangular base where each corner represents the origin of leg

    # def plot_base(self, ax, rot=[0,0,0], is_radians=True, center_offset=[0,0,0]):
    #     # get coordinates
    #     p = (self.base_pose(rot, is_radians, center_offset)).transpose()     
    #     # plot coordinates
    #     ax.plot3D(asarray(p[0,:]).flatten(), asarray(p[1,:]).flatten(), asarray(p[2,:]).flatten(), 'r')
    #     return
       
    # # plot leg 
    # def plot_leg(self, ax, xyz, rot=[0,0,0], legID=0, is_radians=True, center_offset=[0,0,0]):
    #     # get coordinates
    #     p = ((self.leg_pose(xyz, rot, legID, is_radians, center_offset) \
    #             + self.base_pose(rot,is_radians,center_offset)[legID]).transpose())
    #     # plot coordinates
    #     ax.plot3D(asarray(p[0,:]).flatten(), asarray(p[1,:]).flatten(), asarray(p[2,:]).flatten(), 'b')
    #     return

    # def plot_robot(self, xyz, rot=[0,0,0], leg_N=4, is_radians=True, limit=0.250, center_offset=[0,0,0]):
    
    #     ax = self.ax_view(limit)  # set the view
    #     self.plot_base(ax,rot, is_radians, center_offset)  # plot base

    #     # plot legs
    #     for leg in range(leg_N):
    #         self.plot_leg(ax,xyz[leg],rot,leg, is_radians, center_offset) 
        
    #     # show figure
    #     plt.show()
    #     return

        
    # TO-DO : modify this function depending on your robot's configuration
    # adjusting angle for specific configurations of motors, incl. orientation
    # this will vary for each robot (possibly for each leg as well)
    def angle_corrector(self, angles=[0,0,0], is_right=True):
        angles[1] -= 1.5*pi; # add offset 
        
        if is_right:
            theta_1 = angles[0] - pi
            theta_2 = angles[1] + 45*pi/180 # 45 degrees initial offset
        else: 
            if angles[0] > pi:  
                theta_1 = angles[0] - 2*pi
            else: theta_1 = angles[0]
            
            theta_2 = -angles[1] - 45*pi/180
        
        theta_3 = -angles[2] + 45*pi/180
        return [theta_1, theta_2, theta_3]
        
    # set view  
    # @staticmethod
    # def ax_view(limit):
    #     ax = plt.axes(projection="3d")
    #     ax.set_xlim(-limit, limit)
    #     ax.set_ylim(-limit, limit)
    #     ax.set_zlim(-limit, limit)
    #     ax.set_xlabel("X")
    #     ax.set_ylabel("Y")
    #     ax.set_zlabel("Z")
    #     return ax

def main(args=None):
    rclpy.init(args=args)
    node = kinematics()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


