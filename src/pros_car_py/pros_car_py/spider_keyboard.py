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
from pros_car_py import spider_utils


_FORWARD_ROUTINE_ANGLES: dict[str, float] = {
    "shoulder_down": 0,
    "shoulder_flat": 15,
    "shoulder_up": 40.0,
    "calf_front": 35.0,
    "calf_back": -35.0
}

FORWARD_ACTION: dict[str, list[float]] = {
    "0": [
        _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,
        _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,

        _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,
        _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,

        _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,
        _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,

        _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,
        _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,
    ],

    "1": [
        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],
        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],

        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],
        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],

        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],
        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],

        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],
        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],

    ],

    "2": [
        _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,
        _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,

        _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,
        _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,

        _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,
        _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,

        _FORWARD_ROUTINE_ANGLES["shoulder_down"], 0,
        _FORWARD_ROUTINE_ANGLES["shoulder_up"], 0,

    ],

    "3": [
        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],
        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],

        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],
        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],

        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],
        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],

        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_back"],
        _FORWARD_ROUTINE_ANGLES["shoulder_flat"], _FORWARD_ROUTINE_ANGLES["calf_front"],
]  
}


class SpiderKeyboardController(Node):
    def __init__(self, stdscr):
        super().__init__('Spider_keyboard')

        self.joint_trajectory_publisher_ = self.create_publisher(
            JointTrajectoryPoint,
            'spider_joint_trajectory_point',
            10
        )

        """0/1: LFF, 2/3: LF, 4/5: LB, 6/7: LBB, 8/9:RFF, 10/11: RF, 12/13: RB, 14/15: RBB"""
        self.joint_pos = [0.0] * 16
        self.standing_pos = self.joint_pos

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

    def run(self):
        self.stdscr.nodelay(True)
        try:
            while rclpy.ok():
                c = self.stdscr.getch()

                # Check if a key was actually pressed
                if c != curses.ERR:
                    self.key_in_count += 1
                    self.print_basic_info(c)
                    #  以下都是機械手臂
                    if c == ord('0'):
                        self.handle_key_0()
                    elif c == ord('1'):
                        self.handle_key_1()
                    elif c == ord('2'):
                        self.handle_key_2()
                    elif c == ord('3'):
                        self.handle_key_3()
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

    def handle_key_0(self):
        self.stdscr.addstr(f"spider walk...")
        self.joint_pos = spider_utils.convert_human_to_unity_spider_joint_angles(FORWARD_ACTION["0"])
        self.pub_arm()

        pass

    def handle_key_1(self):
        self.stdscr.addstr(f"spider walk...")
        self.joint_pos = spider_utils.convert_human_to_unity_spider_joint_angles(FORWARD_ACTION["1"])
        self.pub_arm()
        pass

    def handle_key_2(self):
        self.stdscr.addstr(f"spider walk...")
        self.joint_pos = spider_utils.convert_human_to_unity_spider_joint_angles(FORWARD_ACTION["2"])
        self.pub_arm()
        pass

    def handle_key_3(self):
        self.stdscr.addstr(f"spider walk...")
        self.joint_pos = spider_utils.convert_human_to_unity_spider_joint_angles(FORWARD_ACTION["3"])
        self.pub_arm()

    def handle_key_b(self):
        self.stdscr.addstr(f"spider return to origin state...")
        self.joint_pos = self.standing_pos
        self.pub_arm()
        pass

    # transfer a "degree 2D array" to "radians 2D array"
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
    

# ... Rest of your code, e.g. initializing rclpy and running the node
def main(args=None):
    rclpy.init(args=args)
    stdscr = curses.initscr()
    node = SpiderKeyboardController(stdscr)

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
