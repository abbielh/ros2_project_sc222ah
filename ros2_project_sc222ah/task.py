# Exercise 1 - Display an image of the camera feed to the screen

# from __future__ import division
import threading
import sys, time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.task import Future
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from rclpy.exceptions import ROSInterruptException
from math import sin, cos
import signal
import random


class Robot(Node):
    def __init__(self):
        super().__init__("Robot")

        self.action_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.goal_handle = None
        self.goal_reached = False

    def send_goal(self):
        x = random.uniform(9.7, -15.7)
        y = random.uniform(-12, 6.42)
        yaw = 0.0
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        goal_msg.pose.pose.orientation.z = sin(yaw / 2)
        goal_msg.pose.pose.orientation.w = cos(yaw / 2)

        self.action_client.wait_for_server()
        self.send_goal_future = self.action_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected")
            return

        self.get_logger().info("Goal accepted")
        self.goal_handle = goal_handle
        print(f"GOAL RESPOSE:goal handle {self.goal_handle}")
        self.goal_reached = False
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result == None:
            self.goal_handle.status = GoalStatus.STATUS_ABORTED

        self.get_logger().info(f"Navigation result: {result}")
        print(f"RESULT CALLBACK:goal handle {self.goal_handle}")
        if (
            self.goal_handle.status == GoalStatus.STATUS_SUCCEEDED
            or self.goal_handle.status == GoalStatus.STATUS_ABORTED
        ):
            self.send_goal()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def cancel_current_goal(self):
        if self.goal_handle.status != GoalStatus.STATUS_CANCELED:
            future = self.goal_handle.cancel_goal_async()
            if future.result():  # Check if cancellation was successful
                self.get_logger().info("Goal successfully canceled.")
                self.goal_handle = future.result()
            else:
                self.get_logger().warn("Goal cancellation failed.")


class colourID(Node):
    def __init__(self, robot):
        super().__init__("colourID")

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.callback, 10
        )
        self.sensitivity = 10
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.blue_found = False
        self.robot = robot

    def callback(self, data):
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        hsv_blue_lower = np.array([120 - self.sensitivity, 100, 100])
        hsv_blue_upper = np.array([120 + self.sensitivity, 255, 255])

        Hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        blue_mask = cv2.inRange(Hsv_image, hsv_blue_lower, hsv_blue_upper)

        contours, _ = cv2.findContours(
            blue_mask, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_SIMPLE
        )

        if len(contours) > 0:
            c = max(contours, key=cv2.contourArea)

            if cv2.contourArea(c) > 1000:  # <What do you think is a suitable area?>
                print(cv2.contourArea(c))
                (x, y), radius = cv2.minEnclosingCircle(c)
                centre = (int(x), int(y))
                radius = int(radius)
                cv2.circle(image, centre, radius, (0, 0, 255), 5)
                self.blue_found = True

        desiredVelo = Twist()
        if self.blue_found == True:
            self.robot.cancel_current_goal()
            if centre[0] < 490:
                desiredVelo.angular.z = 0.05
            elif centre[0] > 510:
                desiredVelo.angular.z = -0.05
            elif cv2.contourArea(c) <= 100000:
                print("forward")
                desiredVelo.linear.x = 0.2
            self.publisher.publish(desiredVelo)

        cv2.namedWindow("camera_Feed", cv2.WINDOW_NORMAL)
        cv2.imshow("camera_Feed", image)
        cv2.resizeWindow("camera_Feed", 320, 240)
        cv2.waitKey(3)


# Create a node of your class in the main and ensure it stays up and running
# handling exceptions and such
def main():

    def signal_handler(sig, frame):
        rclpy.shutdown()

    # Instantiate your class
    # And rclpy.init the entire node
    rclpy.init(args=None)
    robot = Robot()
    colourid = colourID(robot)

    signal.signal(signal.SIGINT, signal_handler)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(robot)
    executor.add_node(colourid)

    robot.send_goal()

    executor.spin()

    try:
        while rclpy.ok():
            continue

    except ROSInterruptException:
        pass

    # Remember to destroy all image windows before closing node
    cv2.destroyAllWindows()


# Check if the node is executing in the main path
if __name__ == "__main__":
    main()
