#!/usr/bin/env python3
import rospy, cv2, cv_bridge, numpy, math
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
import moveit_commander
from q_learning_project.msg import RobotMoveDBToBlock
import time
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import keras_ocr
import os

three = ["3"]
two = ["2", "s"]
one = ["1", "1l", "l"]

class RobotAction(object):

    def __init__(self, robot_dumbbell, goal_block_num):
        self.robot_db = robot_dumbbell
        if goal_block_num == 1: goal_block_num = one
        elif goal_block_num == 2: goal_block_num = two
        elif goal_block_num == 3: goal_block_num = three
        self.goal_block_num = goal_block_num

    def __str__(self):
        output_str = ("Robot action: move " + self.robot_db.upper() + " to block " + str(self.goal_block_num))
        return output_str

class RobotMovement(object):
    def __init__(self):
        # SUBSCRIBERS/PUBLISHERS
        # ======================
        # set-up keras and cv libraries
        self.pipeline = keras_ocr.pipeline.Pipeline()
        self.bridge = cv_bridge.CvBridge()
        # camera
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.update_image)
        # lidar
        rospy.Subscriber("scan", LaserScan, self.update_distance)
        # arm and gripper
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        # movement
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=100)

        time.sleep(5) # give it a bit of time!
        print("Finished initializing subscribers/publishers!")

        # PHASE SYSTEM
        # ============
        # We use a "phase" system paradigm to perform the movement portion
        # of this assignment. Splitting into phases allowed us to modularize
        # and work on different components at the same time.
        # When a phase is actively being executed it is set to -1. 
        # When it completes a phase, it will move on to the next phase.
        # 
        # Phase 0 - Move towards a colored dumbbell
        # Phase 1 - Pick up the dumbbell 
        # Phase 2 - Move towards numbered block
        # Phase 3 - Put down the dumbbell
        # Phase 4 - Reset and move to Phase 0.
        self.phase = 0

        # TURTLEBOT STATES
        # ================
        # RGB camera image data var
        self.image_data = None
        # camera cv/ocr vars
        self.new_object = False
        self.found_box = False
        # lidar distance
        self.distance = 0
        # arm and gripper
        arm_start = [0, .85, -.3, -.35]
        grip_start = [0.016, 0.016]
        self.move_group_arm.go(arm_start)
        self.move_group_gripper.go(grip_start)
        # movement
        self.twist = Twist()
        # goal states (dumbbell and block goal)
        self.robot_action_queue = []
        self.current_action = 0
        self.occupied_blocks = []
        # self.selected_dumbbell = ""
        # self.block_goal = 0
        # takes in q_matrix and updates goal states
        self.load_in_actions_and_matrix()
        print(f"Loaded in q matrix: [{[str(act) for act in self.robot_action_queue]}]")
        
        # initialize this node
        rospy.init_node('turtlebot3_movement')
        print("Finished initialization.")

        # TURTLEBOT EXECUTE
        # ================
        time.sleep(5)
        # run execute
        self.execute()

    def execute(self):
        while len(self.robot_action_queue) > 0:
            self.phase = 0
            self.phase_router()
            

    def phase_router(self):
        if self.phase == 0:
            print("PHASE 0")
            self.execute_phase_0()
        elif self.phase == 1:
            print("PHASE 1")
            self.execute_phase_1()
        elif self.phase == 2:
            print("PHASE 2")
            self.execute_phase_2()
        elif self.phase == 3:
            print("PHASE 3")
            self.execute_phase_3()
        elif self.phase == 4:
            print("PHASE 4")
            self.execute_phase_4()
        else:
            print("PHASE ERROR")
            return

    def execute_phase_4(self):
        """ Puts down the dumbbell
            (Puts it down carefully, so it won't topple.)
        """
        arm = {
            "back0": [0.65, 0.25, -0.3, -0.5],
            "back1": [0.3, 0.5, -0.3, -0.5],
            "back2": [0.3, 0.5, -0.3, .25],
            "back3": [0.3, 0.5, -0.1, .25],
            "reset": [0, .85, -.3, -.35]
        }
        move_arm = self.move_group_arm

        print("Now putting arm down!")
        move_arm.go(arm["back0"], wait=True)
        move_arm.stop()
        move_arm.go(arm["back1"], wait=True)
        move_arm.stop()
        move_arm.go(arm["back2"], wait=True)
        move_arm.stop()
        move_arm.go(arm["back3"], wait=True)
        move_arm.stop()

        # reset arm to original
        move_arm.go(arm["reset"], wait=True)
        move_arm.stop()
        # slowly back away
        self.twist.linear.x = -0.05
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(10)
        # back away quicker
        self.twist.linear.x = -0.2
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(5)
        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)

        self.current_action += 1
        print(f"Current action is now {self.current_action}")
        print(f"Action: {self.robot_action_queue[self.current_action]}")
        return

    def execute_phase_3(self):
        """ Moves in the direction of the identified block
            This assumes we are already pointed towards the block.
        """

        while(True):
            image = self.bridge.imgmsg_to_cv2(self.image_data,desired_encoding='bgr8')

            if self.found_box:
                # Runs if we have detected the correct number on the box
                # print("heading to block")
                # looking for black
                lower = numpy.array([ 0, 0, 0])
                upper = numpy.array([179, 255, 20])
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower, upper)
                # we now erase all pixels that aren't of the selected color
                h, w, d = image.shape
                search_top = int(0)
                search_bot = int(h)
                mask[0:search_top, 0:w] = 0
                mask[search_bot:h, 0:w] = 0
                # using moments() function, determine the center of the colored pixels
                M = cv2.moments(mask)
                # if there are any colored pixels found
                if M['m00'] > 0:
                    # print("found colored pixels")
                    # determine the center of the colored pixels in the image
                    cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                    cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                    k_p, err = .01, w/2 - cx
                    # print(f"distance: {self.distance}")
                    if self.distance < .5:
                        # print("im close")
                        self.twist.linear.x = 0
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                        self.found_box = False
                        self.phase = 4
                        self.phase_router()
                        return
                    else:
                        # print("im too far")
                        self.twist.linear.x = 0.1
                        self.twist.angular.z = k_p * err *.01
                        self.cmd_vel_pub.publish(self.twist)


    def execute_phase_2(self):
        """ Uses OCR image recognition to figure out what direction
            the numbered block is in
        """
        while(True):
            image = self.bridge.imgmsg_to_cv2(self.image_data,desired_encoding='bgr8')

            if self.new_object:
                h, w, d = image.shape
                print(f"Image size is h:{h} w:{w} d:{d}")

                # crops off everything but the center
                x_offset, y_offset = int(w/3), int(h/3)
                for i in range(d):
                    for j in range(y_offset, h-y_offset):
                        for k in range(x_offset, w-x_offset):
                            image[j][k][i] = 255

                # Runs if we have found a new object to perform ocr on
                print("performing ocr on new object")
                images = [image]
                prediction_groups = self.pipeline.recognize(images)
                print(f"{prediction_groups}")

                for num, box in prediction_groups[0]:
                    goal_nums = self.robot_action_queue[self.current_action].goal_block_num
                    print(f"The OCR detected the character: {num}")
                    print(f"We are looking for: {goal_nums}")

                    if num in self.occupied_blocks:
                        break
                    elif num in goal_nums:
                        print("FOUND THE BOX!")
                        self.occupied_blocks += goal_nums
                        self.found_box = True
                        self.new_object = False
                        self.phase = 3
                        self.phase_router()
                        return
                print(f"did not find character")
                self.twist.angular.z = .1 * (-1 ** self.current_action)
                self.cmd_vel_pub.publish(self.twist)
                time.sleep(3)
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                self.new_object = False

            else:
                # Runs if we are looking for a new object
                # print("looking for a new object")
                # looking for black
                lower = numpy.array([ 0, 0, 0])
                upper = numpy.array([179, 255, 20])
                hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, lower, upper)
                # we now erase all pixels that aren't of the selected color
                h, w, d = image.shape
                search_top = int(0)
                search_bot = int(h)
                mask[0:search_top, 0:w] = 255
                mask[search_bot:h, 0:w] = 255
                # using moments() function, determine the center of the colored pixels
                M = cv2.moments(mask)
                # if there are any colored pixels found
                if M['m00'] > 0:
                    # print("found colored pixels")
                    # determine the center of the colored pixels in the image
                    cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                    # cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                    if w/3 < cx < 2*w/3:
                        self.new_object = True
                        self.twist.angular.z = 0
                        self.cmd_vel_pub.publish(self.twist)
                    else:
                        self.twist.angular.z = .1 * (-1 ** self.current_action)
                        self.cmd_vel_pub.publish(self.twist)



                    # k_p, err = .01, w/2 - cx
                    # offset = 30
                    # # print(f"cx:{cx}, leftboundry:{w/2 - offset}, rightboundry: {w/2 + offset}")
                    # if cx > 2*w/3:
                    #     continue
                    # elif cx > w/2 - offset and cx < w/2 + offset:
                    #     # print("centered")
                    #     self.new_object = True
                    #     self.twist.angular.z = 0
                    #     self.cmd_vel_pub.publish(self.twist)
                    # else:
                    #     # print("adjusting")
                    #     self.twist.angular.z = k_p * err * .05
                    #     self.cmd_vel_pub.publish(self.twist)
                else:
                    # print("no colored pixels found")
                    self.twist.angular.z = .15 * (-1 ** self.current_action)
                    self.cmd_vel_pub.publish(self.twist)

    def execute_phase_1(self):
        """ Tells robot to pick up the dumbbell and 
            moves dumbbell out of way of camera
        """
        move_arm = self.move_group_arm
        arm = {
            "lower": [-0.01, .9, -.3, -0.5],
            "upper": [0.3, 0.5, -0.3, -0.5],
            "upper_diag": [0.65, 0.25, -0.3, -0.5],
            "side": [1.5, 0.25, -0.3, -0.5],
        }

        move_arm.go(arm["lower"])
        move_arm.stop()

        # move the arm to a resting position
        print("Now raising the arm up!")
        move_arm.go(arm["upper"], wait=True)
        move_arm.stop()
        move_arm.go(arm["upper_diag"], wait=True)
        move_arm.stop()
        move_arm.go(arm["side"], wait=True)
        move_arm.stop()
        print("Should be holding dumbbell at this point.")

        # Scoot the robot back so it is now closer to the numbered blocks.
        self.twist.linear.x =-.2
        self.cmd_vel_pub.publish(self.twist)
        time.sleep(8)

        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)

        self.phase = 2
        self.phase_router()
        return

    def move_to_col(self, image, lower, upper):
        """ Moves to the color given by the lower and upper HSVs
            Enters phase 1 once it is close enough.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        # we now erase all pixels that aren't of the selected color
        h, w, d = image.shape
        search_top, search_bot = int(0), int(h)
        mask[0:search_top, 0:w], mask[search_bot:h, 0:w] = 0, 0
        # using moments() function, determine the center of the colored pixels
        M = cv2.moments(mask)
        # if there are any colored pixels found
        if M['m00'] > 0:
            # determine the center of the colored pixels in the image
            cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

            # print("Colored pixels were found in the image.")
            print(f"Distance: {self.distance}, cy: {cy}")

            # if blue_cond or green_cond or red_cond:
            dumbbell_close_enough = 305 < cy < 330 or 0.1 < self.distance < 0.2

            if dumbbell_close_enough: 
                # print("Close to dumbbell! Now picking it up.")
                time.sleep(8)
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                self.phase = 1 # Phase 1 executes pick-up
                self.phase_router()
                return
            else:
                # print("Out of range of dumbbells. Moving forward.")
                # pid control variables!
                k_p, err = .01, w/2 - cx
                # alter trajectory accordingly
                self.twist.linear.x = 0.02
                self.twist.angular.z = k_p * err *.02
                self.cmd_vel_pub.publish(self.twist)
        else:
            # print("No colored pixels -- spinning in place.")
            self.twist.linear.x = 0
            self.twist.angular.z = .1
            self.cmd_vel_pub.publish(self.twist)

    def execute_phase_0(self):
        """ Phase 0 is the base state.
            Primarily, it searches for colored pixels and moves to the dumbbell.
            Much of the logic is abstracted to move_to_col.
        """

        while(True):
            # show the debugging window
            image = self.bridge.imgmsg_to_cv2(self.image_data,desired_encoding='bgr8')

            lower = {
                "green": numpy.array([60, 60, 60]),
                "red": numpy.array([0, 190, 160]),
                "blue": numpy.array([94, 80, 2])
            }
            upper = {
                "green": numpy.array([65, 255, 250]),
                "red": numpy.array([2, 255, 255]),
                "blue": numpy.array([126, 255, 255])
            }
            dumbbell = self.robot_action_queue[self.current_action].robot_db
            # print(f"Searching for {dumbbell}")
            if dumbbell == "GREEN":
                self.move_to_col(image, lower["green"], upper["green"])
            elif dumbbell == "RED":
                self.move_to_col(image, lower["red"], upper["red"])
            elif dumbbell == "BLUE":
                self.move_to_col(image, lower["blue"], upper["blue"])
            else:
                print("NO COLOR ERROR")
                exit
    
    def update_distance(self, data):
        self.distance = data.ranges[0]

    def update_image(self, data):
        self.image_data = data



    def load_in_actions_and_matrix(self):
        q_matrix = numpy.loadtxt(os.path.dirname(__file__) + "/saved_matrix/q_matrix.txt", delimiter=" ")
        action_matrix = numpy.loadtxt(os.path.dirname(__file__) + "/action_states/action_matrix.txt")
        colors = ["RED", "GREEN", "BLUE"]
        actions = numpy.loadtxt(os.path.dirname(__file__) + "/action_states/actions.txt")
        actions = list(map(
            lambda x: {"dumbbell": colors[int(x[0])], "block": int(x[1])},
            actions
        ))

        first_state = 0
        first_act = q_matrix[first_state].tolist().index(max(q_matrix[first_state]))
        first_db, first_block = actions[first_act]["dumbbell"], actions[first_act]["block"]
        
        second_state = action_matrix[first_state].tolist().index(first_act)
        second_act = q_matrix[second_state].tolist().index(max(q_matrix[second_state]))
        second_db, second_block = actions[second_act]["dumbbell"], actions[second_act]["block"]
        
        third_state = action_matrix[second_state].tolist().index(second_act)
        third_act = q_matrix[third_state].tolist().index(max(q_matrix[third_state]))
        third_db, third_block = actions[third_act]["dumbbell"], actions[third_act]["block"]

        self.robot_action_queue = [RobotAction(first_db, first_block),
            RobotAction(second_db, second_block), 
            RobotAction(third_db, third_block)]        

    def run(self):
        rospy.spin()
if __name__=="__main__":
    node = RobotMovement()
    node.run()
