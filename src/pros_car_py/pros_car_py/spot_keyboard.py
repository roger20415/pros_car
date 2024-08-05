import time
from rclpy.node import Node
import rclpy
import orjson
import curses
import threading
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint
from pros_car_py.env import *
import math

class SpotKeyboardController(Node):
    def __init__(self, stdscr, vel: float = 10):
        super().__init__('Spot_keyboard')
        self.vel = vel
        self.rotate_angle = math.radians(10.0) # 控制機械手臂每次移動的角度
        self.rotate_speed = vel

        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint,
            'spot_joint_trajectory_point',
            10
        )
        # ros2 topic pub /center_dir std_msgs/msg/String "{data: 'left'}"
        self.publisher = self.create_publisher(
            String,
            "center_dir",  # topic name
            10
        )
        """ [0~2]:LF [3~5]:RF [6~8]:RB [9~11]:LB """
        self.joint_pos = [math.radians(0), math.radians(25), math.radians(-60), math.radians(0), math.radians(-25), math.radians(60),
                          math.radians(0), math.radians(-25), math.radians(60), math.radians(0), math.radians(25), math.radians(-60)]
        
        self.standing_pos = self.joint_pos
        self.forward_First_Time_displacement   = [[0, 10, -50, 0, 0, 0, 0, -10, 50, 0, 0, 0],
                                                  [0, -10, 20, 0, 0, 0, 0, 10, -20, 0, 0, 0],
                                                  [0, -10, 20, 0, 0, 0, 0, 10, -20, 0, 0, 0],
                                                  [0, -20, 20, 0, 0, 0, 0, 20, -20, 0, 0, 0]]

        self.forward_routine_left_displacement = [[0, 10, -50, 0, -10, 5, 0, -10, 50, 0, 10, -5],
                                                  [0, -10, 20, 0, -10, 5, 0, 10, -20, 0, 10, -5],
                                                  [0, -10, 20, 0, -10, 0, 0, 10, -20, 0, 10, 0],
                                                  [0, -20, 20, 0, 0, 0, 0, 20, -20, 0, 0, 0]]
        
        self.forward_routine_right_displacement = [[0, 10, -5, 0, -10, 50, 0, -10, 5, 0, 10, -50],
                                                  [0, 10, -5, 0, 10, -20, 0, -10, 5, 0, -10, 20],
                                                  [0, 10, 0, 0, 10, -20, 0, -10, 0, 0, -10, 20],
                                                  [0, 0, 0, 0, 20, -20, 0, 0, 0, 0, -20, 20]]
        
        self.backward_routine_left_displacement = [[0, -5, 5, 0, 0, 0, 0, 5, -5, 0, 0, 0],
                                                  [0, -5, 5, 0, 0, 0, 0, 5, -5, 0, 0, 0],
                                                  [0, -10, 5, 0, 0, 0, 0, 10, -5, 0, 0, 0],
                                                  [0, 10, -10, 0, 0, 0, 0, -10, 10, 0, 0, 0]]
        
        self.backward_routine_right_displacement = [[0, 0, 0, 0, 5, -5, 0, 0, 0, 0, -5, -5],
                                                  [0, 0, 0, 0, 5, -5, 0, 0, 0, 0, -5, -5],
                                                  [0, 0, 0, 0, 10, -5, 0, 0, 0, 0, -10, -5],
                                                  [0, 0, 0, 0, -10, 10, 0, 0, 0, 0, 10, 10]]

        self.deg2rad_transfer_for_joints(self.forward_routine_left_displacement)
        self.deg2rad_transfer_for_joints(self.forward_routine_right_displacement)
        self.deg2rad_transfer_for_joints(self.backward_routine_left_displacement)
        self.deg2rad_transfer_for_joints(self.backward_routine_right_displacement)
        self.deg2rad_transfer_for_joints(self.forward_First_Time_displacement)

        self.flag_which_leg = False # next move -- False : left , True : right 
        self.flag_switch_forward_backward = False # False: last move is forwarding, True: last move is backwarding.
        self.flag_First_move = True # if this moving is the first move after start or stand command.
        self.step_now = 0

        self.stdscr = stdscr
        curses.noecho()
        curses.raw()
        self.stdscr.keypad(False)
        self.key_in_count = 0
        self._car_state_msg = ""

    def _sub_callback(self, msg):
        # Process the incoming message (if needed)
        # TODO show data in screen
        self._car_state_msg = str(self.get_clock().now()) + " " + msg.data

    def run(self, vel=None):
        if vel is None:
            vel = self.vel
        self.stdscr.nodelay(True)
        try:
            while rclpy.ok():
                c = self.stdscr.getch()

                # Check if a key was actually pressed
                if c != curses.ERR:
                    self.key_in_count += 1
                    self.print_basic_info(c)
                    #  以下都是機械手臂
                    if c == ord('w'):
                        self.handle_key_w()
                    elif c == ord('s'):
                        self.handle_key_s()
                    elif c == ord('a'):
                        self.handle_key_a()
                    elif c == ord('d'):
                        self.handle_key_d()
                    elif c == ord('b'):
                        self.handle_key_b()
                    elif c == ord('q'):  # Exit on 'q'
                        break
                    # self.pub_arm()
                    print()
                else:
                    self.print_basic_info(ord(' '))
                    time.sleep(0.01)

            # origin_string = self.serial.readline()
            # self.stdscr.move(3, 0)
            # self.stdscr.addstr(f"{self.key_in_count:5d} receive: {origin_string} ")

        finally:
            curses.endwin()

    def print_basic_info(self, key):
        # Clear the screen
        self.stdscr.clear()

        self.stdscr.move(0, 0)
        # Print a string at the current cursor position
        self.stdscr.addstr(f"{self.key_in_count:5d} Key '{chr(key)}' pressed!")

        # show receive data
        self.stdscr.move(1, 0)
        self.stdscr.addstr(f"Received msg : {self._car_state_msg}")

        # self.stdscr.move(4, 0)
        # self.stdscr.addstr(f"Arm pos : {self.joint_pos}")
        # self.stdscr.move(5, 0)

        # self.get_logger().debug(f"{self.key_in_count:5d} Key '{chr(key)}' pressed!")

    def clamp(self, value, min_value, max_value):
        """限制值在一定範圍內"""
        return max(min_value, min(value, max_value))

    def handle_key_w(self):
        # self.stdscr.addstr(f"spot move forward")
        # if(self.flag_switch_forward_backward == True):
        #     self.flag_which_leg = not self.flag_which_leg
        #     self.flag_switch_forward_backward = False
        # for i in range(len(self.forward_routine_left_displacement)):
        #     for j in range(len(self.forward_routine_left_displacement[0])):
        #         if(self.flag_which_leg == False):
        #             if(self.flag_First_move == True):
        #                 self.joint_pos[j] += self.forward_First_Time_displacement[i][j]
        #             else:
        #                 self.joint_pos[j] += self.forward_routine_left_displacement[i][j]
        #         else:
        #             self.joint_pos[j] += self.forward_routine_right_displacement[i][j]
        #     self.pub_arm()
        #     time.sleep(0.2)
        # if(self.flag_which_leg == True):
        #     self.flag_which_leg = False
        # else:
        #     self.flag_which_leg = True

        # if(self.flag_First_move == True):
        #     self.flag_First_move = False
        self.pub_center_front()
        pass

    def handle_key_s(self):
        # self.stdscr.addstr(f"spot move backward")
        # if(self.flag_switch_forward_backward == False):
        #     self.flag_which_leg = not self.flag_which_leg
        #     self.flag_switch_forward_backward = True

        # for i in range(len(self.backward_routine_left_displacement)):
        #     for j in range(len(self.backward_routine_left_displacement[0])):
        #         if(self.flag_which_leg == True):
        #             self.joint_pos[j] -= self.backward_routine_left_displacement[i][j]
        #         else:
        #             self.joint_pos[j] -= self.backward_routine_right_displacement[i][j]
        #     self.pub_arm()
        #     time.sleep(0.5)
        # if(self.flag_which_leg == True):
        #     self.flag_which_leg = False
        # else:
        #     self.flag_which_leg = True
        self.pub_center_back()
        pass

    def handle_key_a(self):
        self.pub_center_left()
        pass

    def handle_key_d(self):
        self.pub_center_right()
        pass

    def handle_key_b(self):
        # # initialize
        self.stdscr.addstr(f"spot return to origin state...")
        # self.joint_pos = [ 弧度表示之角度 ]  # 角度（0, 15, -25, 0, -15, 25, 0, -25, 25, 0, 35, -25） degree
        self.joint_pos = [math.radians(0), math.radians(25), math.radians(-60), math.radians(0), math.radians(-25), math.radians(60),
                          math.radians(0), math.radians(-25), math.radians(60), math.radians(0), math.radians(25), math.radians(-60)]
        self.pub_arm()

        
        self.pub_center_origin()
        pass

    def deg2rad_transfer_for_joints(self, displacement):
        for i in range(len(self.forward_routine_left_displacement)):
            for j in range(len(self.forward_routine_left_displacement[0])):
                displacement[i][j] = math.radians(displacement[i][j])

    def pub_arm(self):
        msg = JointTrajectoryPoint()
        # msg.positions = [math.degrees(pos) for pos in self.joint_pos]   # Replace with actual desired positions
        msg.positions = [float(pos) for pos in self.joint_pos]
        msg.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]  # Replace with actual desired velocities
        self.joint_trajectory_publisher_.publish(msg)
    
    def pub_center_left(self):
        control_msg = String()
        control_msg.data = "left"
        self.publisher.publish(control_msg)
    
    def pub_center_right(self):
        control_msg = String()
        control_msg.data = "right"
        self.publisher.publish(control_msg)

    def pub_center_front(self):
        control_msg = String()
        control_msg.data = "front"
        self.publisher.publish(control_msg)
    
    def pub_center_back(self):
        control_msg = String()
        control_msg.data = "back"
        self.publisher.publish(control_msg)

    def pub_center_origin(self):
        control_msg = String()
        control_msg.data = "origin"
        self.publisher.publish(control_msg)

# ... Rest of your code, e.g. initializing rclpy and running the node
def main(args=None):
    rclpy.init(args=args)
    stdscr = curses.initscr()
    vel = 10 if WHEEL_SPEED is None else float(WHEEL_SPEED)
    node = SpotKeyboardController(stdscr, vel=vel)

    # Spin the node in a separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    finally:
        curses.endwin()
        node.get_logger().info(f'Quit keyboard!')
        rclpy.shutdown()
        spin_thread.join()  # Ensure the spin thread is cleanly stopped

if __name__ == '__main__':
    main()
