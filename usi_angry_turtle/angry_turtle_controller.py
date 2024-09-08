#!/usr/bin/env python3
import random
from math import *
from threading import Thread

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_srvs.srv import Empty
from turtlesim.msg import Pose as TurtlesimPose
from turtlesim.srv import SetPen, Kill, Spawn


class TurtleState:
    WRITING_STATE = "writing"
    ANGRY_STATE = "angry"
    POSITIONING_STATE = "positioning"
    FINAL_STATE = "final"


class TurtleBot(Node):
    PEN_ON = SetPen.Request(r=255, g=255, b=255, width=3, off=0)
    PEN_OFF = SetPen.Request(r=0, g=0, b=0, width=0, off=1)

    def __init__(self):
        super().__init__('turtlebot_controller')

        self.velocity_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(TurtlesimPose, '/turtle1/pose', self.turtle_pose, 10)

        self.pose = TurtlesimPose()
        self.pose_target_turtles = dict()

        self.rate = self.create_rate(10)

        self.vel_msg = Twist()

        self.srv_setpen = self.create_client(SetPen, '/turtle1/set_pen')
        self.kill_turtle = self.create_client(Kill, '/kill')
        self.clear = self.create_client(Empty, '/clear')

        self.state = None
        self.total_turtles = 4
        self.turtles_alive = self.total_turtles
        self.goal_turtle_name = None

    def turtle_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose.theta = round(self.pose.theta, 4)

    def turtle_target_pose(self, data, name):
        data.x = round(data.x, 4)
        data.y = round(data.y, 4)
        data.linear_velocity = round(data.linear_velocity, 4)
        self.pose_target_turtles[name] = data

    def sleep(self):
        self.rate.sleep()
        if not rclpy.ok():
            raise rclpy.exceptions.ROSInterruptException

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) +
                    pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=4):
        velocity = constant * self.euclidean_distance(goal_pose)
        return min(max(-5, velocity), 5)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=8):
        return constant * atan2(sin(self.steering_angle(goal_pose) - self.pose.theta),
                                cos(self.steering_angle(goal_pose) - self.pose.theta))

    def angle_difference(self, goal_pose):
        return atan2(sin(goal_pose.theta - self.pose.theta), cos(goal_pose.theta - self.pose.theta))

    def angular_vel_rot(self, goal_pose, constant=12):
        return constant * self.angle_difference(goal_pose)

    def stop_walking(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)

    def go_to_initial_position(self, tolerance=0.1):
        self.srv_setpen.call(self.PEN_OFF)

        init_pose = TurtlesimPose(x=1, y=8, theta=3 * pi / 2)
        while self.euclidean_distance(init_pose) >= tolerance:
            self.move_to_goal(init_pose)

        self.rotate(init_pose)
        self.state = TurtleState.WRITING_STATE

    def rotate(self, rotation_pose, rot_tolerance=0.017):
        while abs(self.angle_difference(rotation_pose)) >= rot_tolerance:
            self.rotate_to_goal(rotation_pose)
        self.stop_walking()

    def writing(self, tolerance=0.1, pursuing_tolerance=2, p=None, pen_offline=None):
        if pen_offline is None:
            pen_offline = [5, 11]

        if p is None:
            p = [TurtlesimPose(x=1, y=5, theta=5 * pi / 3), TurtlesimPose(x=2, y=4, theta=0),
                 TurtlesimPose(x=3, y=4, theta=pi / 6), TurtlesimPose(x=4, y=5, theta=pi / 2),
                 TurtlesimPose(x=4, y=8, theta=0), TurtlesimPose(x=8, y=8, theta=5 * pi / 6),
                 TurtlesimPose(x=5.5, y=7, theta=5 * pi / 4), TurtlesimPose(x=5.5, y=6.3, theta=11 * pi / 6),
                 TurtlesimPose(x=8, y=5.7, theta=7 * pi / 4), TurtlesimPose(x=8, y=5, theta=4 * pi / 3),
                 TurtlesimPose(x=5.5, y=4, theta=0), TurtlesimPose(x=9.5, y=4, theta=pi / 2),
                 TurtlesimPose(x=9.5, y=8, theta=pi / 2)]

        self.srv_setpen.call(self.PEN_ON)
        self.goal_turtle_name = None

        for idx, goal_pose in enumerate(p):
            if idx in pen_offline:
                self.srv_setpen.call(self.PEN_OFF)

            while self.euclidean_distance(goal_pose) >= tolerance:
                self.move_to_goal(goal_pose)
                self.goal_turtle_name = self.get_closer_turtle()

                if self.turtles_alive == 0:
                    continue

                if self.euclidean_distance(self.pose_target_turtles[self.goal_turtle_name]) < pursuing_tolerance:
                    self.srv_setpen.call(self.PEN_OFF)
                    self.get_logger().info(f'Turtle {self.goal_turtle_name} too close')
                    self.state = TurtleState.ANGRY_STATE
                    break

            if self.state == TurtleState.ANGRY_STATE:
                break
            else:
                self.rotate(goal_pose)
                self.srv_setpen.call(self.PEN_ON)

        if self.state != TurtleState.ANGRY_STATE:
            if self.turtles_alive > 0:
                self.get_logger().info('Congratulation, the turtle wrote "USI", but there are some turtles still alive.')
                self.clear.call(Empty.Request())
                self.state = TurtleState.POSITIONING_STATE
            else:
                self.state = None

    def get_closer_turtle(self):
        min_distance = float("inf")
        min_turtle = None

        for name, pose in self.pose_target_turtles.items():
            distance = self.euclidean_distance(pose)

            if distance < min_distance:
                min_distance = distance
                min_turtle = name

        self.goal_turtle_name = min_turtle

        return self.goal_turtle_name

    def get_future_pose(self):
        target_pose = self.pose_target_turtles[self.goal_turtle_name]

        constant = 1.1

        m = constant * target_pose.linear_velocity * self.euclidean_distance(target_pose)

        goal_pose = TurtlesimPose()
        goal_pose.x = target_pose.x + m * cos(target_pose.theta)
        goal_pose.y = target_pose.y + m * sin(target_pose.theta)

        return goal_pose, target_pose

    def become_angry(self, capture_tolerance=0.5):
        self.srv_setpen.call(self.PEN_OFF)

        goal_pose, target_pose = self.get_future_pose()

        while self.euclidean_distance(target_pose) >= capture_tolerance:
            self.move_to_goal(goal_pose)
            goal_pose, target_pose = self.get_future_pose()

        try:
            self.kill_turtle.call(Kill.Request(name=self.goal_turtle_name))
            self.get_logger().info(f'Killed {self.goal_turtle_name}')

            self.turtles_alive -= 1
            del self.pose_target_turtles[self.goal_turtle_name]
            self.goal_turtle_name = None

        except rclpy.ServiceException:
            pass
        self.clear.call(Empty.Request())

        if self.turtles_alive > 0:
            self.state = TurtleState.POSITIONING_STATE
            self.get_logger().info('Returning...')
        else:
            self.state = TurtleState.FINAL_STATE

    def move_to_goal(self, goal_pose):
        self.vel_msg.linear.x = self.linear_vel(goal_pose)
        self.vel_msg.angular.z = self.angular_vel(goal_pose)
        self.velocity_publisher.publish(self.vel_msg)

    def rotate_to_goal(self, goal_pose):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = self.angular_vel_rot(goal_pose)
        self.velocity_publisher.publish(self.vel_msg)

    def main(self):
        thread = TargetsController(self)
        thread.start()

        self.state = TurtleState.POSITIONING_STATE

        while rclpy.ok():
            if self.state == TurtleState.POSITIONING_STATE:
                self.go_to_initial_position()

            if self.state == TurtleState.WRITING_STATE:
                self.writing()

            if self.state == TurtleState.ANGRY_STATE:
                self.become_angry()

            if self.state == TurtleState.FINAL_STATE:
                self.get_logger().info('All turtles have been killed. End of work.')
                break


class TargetsController(Thread):

    def __init__(self, controller):
        Thread.__init__(self)
        self.controller = controller

    def run(self):
        rclpy.spin(self.controller)


def main(args=None):
    rclpy.init(args=args)
    try:
        x = TurtleBot()
        x.main()
    except rclpy.ROSInterruptException:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
