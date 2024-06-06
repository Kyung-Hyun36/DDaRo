#! /usr/bin/env python3

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

class NavigatorNode(Node):
    def __init__(self):
        super().__init__(node_name='gui_basic_navigator')
        self.info('Navigator node started.')
        # initial pose
        self.initial_pose = Pose()

        # current amcl_pose
        self.current_pose = Pose()
        self.current_pose.position.x = 0.0
        self.current_pose.position.y = -4.0

        # nav2 action client
        self.goal_handle = None
        self.result_future = None

        self.waypoint = [
            [-1.0, -2.0],   # fish
            [1.0, -2.0],    # meat
            [-2.0, -2.0],    # vegetable
            [2.0, -2.0],    # daiso
            [-3.0, -2.0],    # counter
            [3.0, -2.0],    # B01
            [-1.0, -1.0],    # B05
            [1.0, -1.0],    # B10
            [-2.0, -1.0],    # B15
            [2.0, -1.0],    # B20
        ]

        # feedback을 이용한 진행률 계산 및 출력
        self.feedback_flag = False
        self.feedback = None

        # 초기 거리
        self.initial_distance = 0.0
        
        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)

        self.initial_pose_received = False
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                       'amcl_pose',
                                                       self._amclPoseCallback,
                                                       amcl_pose_qos)
        
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose',
                                                      10)
        
        # qt gui에서 navigator를 제어하기 위한 subscriber
        self.sub_navigator = self.create_subscription(String, 'navigator', self.navigatorCallback, 10)

        self.control_dict = {
            'go_to_fish': lambda: self.goToPose(0),
            'go_to_meat': lambda: self.goToPose(1),
            'go_to_vegetable': lambda: self.goToPose(2),
            'go_to_daiso': lambda: self.goToPose(3),
            'go_to_counter': lambda: self.goToPose(4),
            'go_to_B01': lambda: self.goToPose(5),
            'go_to_B05': lambda: self.goToPose(6),
            'go_to_B10': lambda: self.goToPose(7),
            'go_to_B15': lambda: self.goToPose(8),
            'go_to_B20': lambda: self.goToPose(9),
        }

    def navigatorCallback(self, msg):
        if msg.data in self.control_dict:
            self.control_dict[msg.data]()
            self.info('Navigation command: ' + msg.data)
        else:
            self.warn('Invalid navigation command: ' + msg.data)

    def setInitialPose(self):
        self.info('setInitialPose')
        self._setInitialPose()
        return

    def goToPose(self, waypoint_num):
        # Sends a `NavToPose` action request and waits for completion
        self.current_waypoint_num = waypoint_num
        self.debug("Waiting for 'NavigateToPose' action server")
        while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
            self.info("'NavigateToPose' action server not available, waiting...")
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = self.waypoint[waypoint_num][0]
        goal_msg.pose.pose.position.y = self.waypoint[waypoint_num][1]

        goal_msg.pose.pose.orientation.w = self.current_pose.orientation.w
        goal_msg.pose.pose.orientation.z = self.current_pose.orientation.z

        self.info('Navigating to goal: ' + str(goal_msg.pose.pose.position.x) + ' ' +
                      str(goal_msg.pose.pose.position.y) + '...')
        self.send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self._feedbackCallback)
        return True

    def _setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        return
    
    # feedback을 이용한 진행률 계산 및 출력
    def getFeedback(self, num):
        if self.feedback_flag == False:
            # 현재 좌표와 목표 좌표의 거리를 구함
            distance = ((self.current_pose.position.x - self.waypoint[num][0]) ** 2 + (self.current_pose.position.y - self.waypoint[num][1]) ** 2) ** 0.5
            self.initial_distance = distance
            self.feedback_flag = True

        self.remaining_distance = self.feedback.distance_remaining / self.initial_distance * 100

        if self.remaining_distance < 0.0:
            self.remaining_distance = 0.0
        elif self.remaining_distance > 100.0:
            self.remaining_distance = 100.0

        return self.remaining_distance

    def _amclPoseCallback(self, msg):
        self.current_pose = msg.pose.pose
        self.initial_pose_received = True
        return
    
    def _feedbackCallback(self, msg):
        self.feedback = msg.feedback
        self.getFeedback(self.current_waypoint_num)
        return

    def info(self, msg):
        self.get_logger().info(msg)
        return

    def warn(self, msg):
        self.get_logger().warn(msg)
        return

    def error(self, msg):
        self.get_logger().error(msg)
        return

    def debug(self, msg):
        self.get_logger().debug(msg)
        return


def main(args=None):
    rclpy.init(args=args)

    navigator = NavigatorNode()
    while True:
        rclpy.spin_once(navigator)
 

if __name__ == '__main__':
    main()