#!/usr/bin/env python3

import rospy, cv2, cv_bridge, numpy

from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan

import moveit_commander

from q_learning_project.msg import RobotMoveDBToBlock

import time

from tf.transformations import quaternion_from_euler, euler_from_quaternion

import keras_ocr

class RobotAction(object):

    def __init__(self, robot_dumbbell, goal_block_num):
        self.robot_db = robot_dumbbell
        self.goal_block_num = goal_block_num

    def __str__(self):
        output_str = ("Robot action: move " + self.robot_db.upper() + " to block " + str(self.goal_block_num))
        return output_str

class RobotMovement(object):

    def __init__(self):

        # initialize this node
        rospy.init_node('turtlebot3_movement')

        # download pre-trained model
        pipeline = keras_ocr.pipeline.Pipeline()

        # ROS subscribe to the topic publishing actions for the robot to take
        rospy.Subscriber("/q_learning/robot_action", RobotMoveDBToBlock, self.prepare_to_take_robot_action)

        # information about the robot action to take
        self.robot_action_queue = [RobotAction("blue", 1)]

        # set up ROS / cv bridge
        self.bridge = cv_bridge.CvBridge()

        # initalize the debugging window
        cv2.namedWindow("window", 1)

        # Vars to store current action goals
        self.selected_dumbbell = ""
        self.block_goal = 0

        # Var to store current phase (0 - go to dumbell, 1 - pick up dumbell, 2 - go to block, 3 - put down dumbell, 4 - reset)
        self.phase = 0

        # subscribe to the robot's RGB camera data stream
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.execute_phase_0)

        # variable to store distance to front object
        self.distance = 0 

        # subscribe to the robot's lidar scanner
        rospy.Subscriber("scan", LaserScan, self.update_distance)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        self.twist = Twist()

    def execute_phase_4(self):
        if self.phase != 4:
            return
        self.phase = -1
        self.robot_action_queue.pop(0)
        self.phase = 0
    
    def execute_phase_3(self):
        if self.phase != 3:
            return
        self.phase = -1
        time.sleep(0.5)

        # move the arm to a position to drop the dumbbell
        # TODO: find better values for this
        arm_joint_goal = [0.0,
                     math.radians(5.0),
                     math.radians(10.0),
                     math.radians(-20.0)]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

        # open the gripper
        # TODO: find better values for this
        gripper_joint_goal = [0.009,0.0009]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        # move the arm to the starting position
        # TODO: find better values for this
        arm_joint_goal = [0.0,
                     math.radians(5.0),
                     math.radians(10.0),
                     math.radians(-20.0)]
        self.move_group_arm.go(arm_joint_goal, wait=True)

        self.phase = 4

    def execute_phase_2(self, msg):
        if self.phase != 2:
            return
        self.phase = -1
        time.sleep(0.5)
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        images = [image]
        prediction_groups = pipline.recognize(images)
        for num, box in prediction_groups[0]:
            if num == self.block_goal:
                # TODO: check that box is centered
                print(box)
                self.twist.linear.x = 0.1
                # TODO: set appropriate value
                while self.distance > 1:
                    timer.sleep(.5)
                self.twist.linear.x = 0
                self.phase = 3
                return
        self.twist.angular.z = .1
        timer.sleep(.5)
        self.twist.angular.z = 0
        self.phase = 2

    def execute_phase_1(self):
        if self.phase != 1:
            return
        self.phase = -1

        time.sleep(0.5)

        # open the gripper
        # TODO: find better values for this
        gripper_joint_goal = [0.009,0.0009]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        # move the arm to the dumbbell
        # TODO: find better values for this
        arm_joint_goal = [0.0,
                     math.radians(5.0),
                     math.radians(10.0),
                     math.radians(-20.0)]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

        # close the gripper
        # TODO: find better values for this
        gripper_joint_goal = [0.009,0.0009]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_gripper.stop()

        # move the arm to a resting position
        # TODO: find better values for this
        arm_joint_goal = [0.0,
                     math.radians(5.0),
                     math.radians(10.0),
                     math.radians(-20.0)]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()

        self.phase = 2

        

    def execute_phase_0(self, msg):
        # check if phase is correct
        if self.phase == 1:
            execute_phase_1()
            return
        elif self.phase == 2:
            execute_phase_2(msg)
            return
        elif self.phase == 3:
            execute_phase_3()
            return
        elif self.phase == 4:
            execute_phase_4()
            return
        elif self.phase != 0:
            return

        time.sleep(0.5)

        if (len(self.robot_action_queue) > 0):

            robot_action_to_take = self.robot_action_queue[0]
            print(robot_action_to_take)

            self.selected_dumbbell = robot_action_to_take.robot_db.upper()
            self.block_goal = robot_action_to_take.goal_block_num

            # converts the incoming ROS message to cv2 format and HSV (hue, saturation, value)
            image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            if self.selected_dumbbell == "GREEN":
                # TODO: set correct ranges
                lower = numpy.array([ 10, 10, 10])
                upper = numpy.array([255, 255, 250])
            elif self.selected_dumbbell == "BLUE":
                # TODO: set correct ranges
                lower = numpy.array([ 10, 10, 10])
                upper = numpy.array([255, 255, 250])
            elif self.selected_dumbbell == "RED":
                # TODO: set correct ranges
                lower = numpy.array([ 10, 10, 10])
                upper = numpy.array([255, 255, 250])
            else:
                print("NO COLOR ERROR")
                exit

            mask = cv2.inRange(hsv, lower, upper)

            # we now erase all pixels that aren't of the selected color
            h, w, d = image.shape
            search_top = int(3*h/4)
            search_bot = int(3*h/4 + 20)
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0

            # using moments() function, determine the center of the colored pixels
            M = cv2.moments(mask)
            # if there are any yellow pixels found
            if M['m00'] > 0:
                # determine the center of the colored pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                # visualize a red circle in our debugging window to indicate
                # the center point of the colored pixels
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                # proportional control to have the robot move towards the center of the colored pixels
                err = w/2 - cx
                k_p = 1.0 / 100.0
                # we check to see if the dumbell is too close to us and adjust accordingly
                # TODO: Pick better distance values
                if self.distance:
                    self.twist.linear.x = -0.05
                elif self.distance > .5 and self.distance < 1:
                    self.twist.linear.x = 0.00
                    self.phase = 1
                else:
                    self.twist.linear.x = 0.1
                self.twist.angular.z = k_p * err
                self.cmd_vel_pub.publish(self.twist)
            else:
                self.twist.angular.z = .1

            # show the debugging window
            cv2.imshow("window", image)
            cv2.waitKey(3)

    def prepare_to_take_robot_action(self, data):
        # add action to queue
        self.robot_action_queue.append(RobotAction(data.robot_db, data.block_id))

    def update_distance(self, data):
        time.sleep(0.5)

        # update distance to objcet in front of the robot
        self.distance = data.ranges[0]


    def run(self):
        rospy.spin()

if __name__=="__main__":

    node = RobotMovement()
    node.run()

