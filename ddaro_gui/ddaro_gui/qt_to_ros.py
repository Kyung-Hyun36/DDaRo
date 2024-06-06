import rclpy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from PyQt5.QtWidgets import *
from PyQt5.QtCore import QThread

class ROSNode(QThread):

    def __init__(self):
        super().__init__()
        self._is_running = True

    def run(self):
        self.node = rclpy.create_node('ddaro_gui_node')
        
        self.sub_cmd_vel = self.node.create_subscription(Twist, 'set_vel', self.cmd_vel_callback, 10)
        self.set_vel = None

        self.sub_pose = self.node.create_subscription(PoseWithCovarianceStamped, 
        'amcl_pose', self.pose_callback, 10)
        self.current_pose_x = 1.0
        self.current_pose_y = -4.0

        self.subscription = self.node.create_subscription(Image, '/camera/camera/color/image_raw', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        self.br = CvBridge()
        self.current_frame = None

        self.pub_navigator = self.node.create_publisher(String, 'navigator', 10)

        while rclpy.ok() and self._is_running:
            rclpy.spin_once(self.node)
        self.node.destroy_node()
        rclpy.shutdown()

    def cmd_vel_callback(self, msg):
        self.set_vel = msg

    def pose_callback(self, msg):
        self.current_pose_x = msg.pose.pose.position.x
        self.current_pose_y = msg.pose.pose.position.y

    def listener_callback(self, msg):
        self.current_frame = self.br.imgmsg_to_cv2(msg, 'bgr8')