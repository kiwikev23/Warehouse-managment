    #! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.node import Node

from rclpy.duration import Duration
from ebot_docking.srv import DockSw
from rclpy.executors import MultiThreadedExecutor


"""
Basic navigation demo to go to pose.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    # initial_pose.pose.orientation.z =     
    initial_pose.pose.orientation.w = 0.0
    navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose
    goal_pose1 = PoseStamped()
    goal_pose1.header.frame_id = 'map'
    goal_pose1.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose1.pose.position.x = 2.1
    goal_pose1.pose.position.y = -6.8   
    goal_pose1.pose.orientation.w = 0.707
    goal_pose1.pose.orientation.z = -0.707

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    navigator.goToPose(goal_pose1)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=500.0):
                goal_pose1.pose.position.x = -3.0
                navigator.goToPose(goal_pose1)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
        node = rclpy.create_node('dock_sw_client')
        client = node.create_client(DockSw, 'dock_control')


        while not client.wait_for_service(timeout_sec=1.0):
            node.get_logger().info('Service not available, waiting again...')

        request = DockSw.Request()
        # request.linear_dock = True
        # request.orientation_dock = True
        request.orientation = 1.57
        request.docking = True 
        request.id = '3' # Set this to True for docking
        response = DockSw.Response()

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node,future)
        future.result()
        print(response)
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')



    goal_pose2 = PoseStamped()
    goal_pose2.header.frame_id = 'map'
    goal_pose2.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose2.pose.position.x = 1.35
    goal_pose2.pose.position.y = -4.16
    goal_pose2.pose.orientation.w = 0.707
    goal_pose2.pose.orientation.z =  0.707
   
    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    navigator.goToPose(goal_pose2)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                navigator.cancelTask()

            # Some navigation request change to demo preemption
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=500.0):
                goal_pose1.pose.position.x = -3.0
                navigator.goToPose(goal_pose1)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
        node2 = rclpy.create_node('dock_sw_client2')
        client2 = node2.create_client(DockSw, 'dock_control')


        while not client2.wait_for_service(timeout_sec=1.0):
            node2.get_logger().info('Service not available, waiting again...')

        print("\nDetaching!!")
        request = DockSw.Request()
        # request.linear_dock = True
        # request.orientation_dock = True
        request.orientation = -1.57
        request.docking = False  # Set this to True for docking
        request.id = '3'
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node2,future)
        future.result()
        # client2.call_async(request)
       
        # rclpy.spin_until_future_complete(node,future)
        # future.result()
        # else:
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')






    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()