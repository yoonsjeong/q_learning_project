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

three = ["3", "s", "e", "5"]
two = ["2"]
one = ["1", "l", "1l"]
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
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.execute_phase_0)
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
        self.selected_dumbbell = ""
        self.block_goal = 0
        # takes in q_matrix and updates goal states
        self.load_in_actions_and_matrix()
        print(f"Loaded in q matrix: [{[str(act) for act in self.robot_action_queue]}]")
        
        # initialize this node
        rospy.init_node('turtlebot3_movement')
        print("Finished initialization.")

    def execute_phase_4(self):
        """ Resets the robot and pops the action from the queue.
        """
        if self.phase != 4:
            return
        self.phase = -1
        self.robot_action_queue.pop(0)
        self.phase = 0
    def execute_phase_3(self):
        """ Puts down the dumbbell
            (Puts it down carefully, so it won't topple.)
        """
        if self.phase != 3:
            return
        self.phase = -1
        arm = {
            "back0": [0.65, 0.25, -0.3, -0.5],
            "back1": [0.3, 0.5, -0.3, -0.5],
            "back2": [0.3, 0.5, -0.3, .25],
            "back3": [0.3, 0.5, -0.1, .25]
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

        self.phase = 4
    def execute_phase_2(self, image):
        """ Uses OCR image recognition to figure out what direction
            the numbered block is in
        """
        if self.phase != 2:
            return
        self.phase = -1

        if self.found_box:
            # Runs if we have detected the correct number on the box
            print("heading to block")
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
                print("found colored pixels")
                # determine the center of the colored pixels in the image
                cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                k_p, err = .01, w/2 - cx
                print(f"distance: {self.distance}")
                if self.distance < 2:
                    print("im close")
                    self.twist.linear.x = 0
                    self.twist.angular.z = 0
                    self.cmd_vel_pub.publish(self.twist)
                    self.found_box = False
                    self.phase = 3
                    print("my dad is so proud")
                    return
                else:
                    print("im too far")
                    self.twist.linear.x = 0.1
                    self.twist.angular.z = k_p * err *.01
                    self.cmd_vel_pub.publish(self.twist)
                    self.phase = 2
                    return

        elif self.new_object:
            # Runs if we have found a new object to perform ocr on
            print("performing ocr on new object")
            images = [image]
            prediction_groups = self.pipeline.recognize(images)
            print(f"{prediction_groups}")
            for num, box in prediction_groups[0]:
                print(f"I detected a {num}")
                if num in self.block_goal:
                    print("FOUND THE BOX!")
                    self.found_box = True
                    self.new_object = False
                    self.phase = 2
                    return
            print("did not find the box")
            self.twist.angular.z =.1
            self.cmd_vel_pub.publish(self.twist)
            time.sleep(5)
            self.twist.angular.z = 0
            self.cmd_vel_pub.publish(self.twist)
            print("done")
            self.new_object = False
            self.phase = 2

        else:
            # Runs if we are looking for a new object
            print("looking for a new object")
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
                print("found colored pixels")
                # determine the center of the colored pixels in the image
                cx, cy = int(M['m10']/M['m00']), int(M['m01']/M['m00'])
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
                k_p, err = .01, w/2 - cx
                offset = 50
                print(f"cx:{cx}, leftboundry:{w/2 - offset}, rightboundry: {w/2 + offset}")
                if cx > w/2 - offset and cx < w/2 + offset:
                    print("centered")
                    self.new_object = True
                    self.twist.angular.z = 0
                    self.cmd_vel_pub.publish(self.twist)
                else:
                    print("adjusting")
                    self.twist.angular.z = k_p * err * .05
                    self.cmd_vel_pub.publish(self.twist)
            else:
                print("no colored pixels found")
                self.twist.angular.z = .1
                self.cmd_vel_pub.publish(self.twist)
            self.phase = 2

        

    def execute_phase_1(self):
        """ Tells robot to pick up the dumbbell and 
            moves dumbbell out of way of camera
        """
        if self.phase != 1:
            return
        self.phase = -1 # closes phase off from being re-called
        time.sleep(0.5)
        print("Entered phase 1")

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
        time.sleep(5)

        self.twist.linear.x = 0
        self.cmd_vel_pub.publish(self.twist)

        print("Ready for Phase 2.")
        self.phase = 2

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
            # draw the circle!
            # cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
            # cv2.imshow("window", image)
            # cv2.waitKey(3)

            print("Colored pixels were found in the image.")
            print(f"Distance: {self.distance}")
            print(f"cy: {cy}, h: {h}")

            # if blue_cond or green_cond or red_cond:
            # blue_cond  = self.selected_dumbbell == "BLUE" and 0.5 < self.distance < 0.73
            # green_cond = self.selected_dumbbell == "GREEN" and 0.5 < self.distance < 1.2
            # red_cond   = self.selected_dumbbell == "RED" and 305 < cy < 330
            dumbbell_close_enough = 305 < cy < 330

            if dumbbell_close_enough: 
                print("Close to dumbbell! Now picking it up.")
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.cmd_vel_pub.publish(self.twist)
                self.phase = 1 # Phase 1 executes pick-up
            else:
                print("Out of range of dumbbells. Moving forward.")
                # pid control variables!
                k_p, err = .01, w/2 - cx
                # alter trajectory accordingly
                self.twist.linear.x = 0.02
                self.twist.angular.z = k_p * err *.01
                self.cmd_vel_pub.publish(self.twist)
        else:
            print("No colored pixels -- spinning in place.")
            self.twist.linear.x = 0
            self.twist.angular.z = .1
            self.cmd_vel_pub.publish(self.twist)

    def execute_phase_0(self, msg):
        """ Phase 0 is the base state.
            Primarily, it searches for colored pixels and moves to the dumbbell.
            Much of the logic is abstracted to move_to_col.
        """
        # show the debugging window
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        # cv2.imshow("window", image)
        # cv2.waitKey(3)
        # route to phase
        if self.phase == 1:
            self.execute_phase_1()
            return
        elif self.phase == 2:
            self.execute_phase_2(image)
            return
        elif self.phase == 3:
            self.execute_phase_3()
            return
        elif self.phase == 4:
            self.execute_phase_4()
            return
        elif self.phase != 0:
            return
        print(f"executing phase 0: q size {len(self.robot_action_queue)}")
        time.sleep(0.5)

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

        if (len(self.robot_action_queue) > 0):
            robot_action_to_take = self.robot_action_queue[0]
            print(robot_action_to_take)
            
            self.selected_dumbbell = robot_action_to_take.robot_db
            self.block_goal = robot_action_to_take.goal_block_num
            
            dumbbell = self.selected_dumbbell.upper()

            print(f"Searching for {dumbbell}")
            if dumbbell == "GREEN":
                self.move_to_col(image, lower["green"], upper["green"])
            elif dumbbell == "RED":
                self.move_to_col(image, lower["red"], upper["red"])
            elif dumbbell == "BLUE":
                self.move_to_col(image, lower["blue"], upper["blue"])
            else:
                print("NO COLOR ERROR")
                exit
        else:
            print("Robot action queue was empty.")
    
    def update_distance(self, data):
        time.sleep(0.5)

        
        # ranges_to_avg = [-1, 0, 1] #[-10, -5, 0, 5, 10]
        
        # total = 0
        # count = 0
        # for rng in ranges_to_avg:
        #     if data.ranges[rng] != float("inf"):
        #         total += data.ranges[rng]
        #         count += 1
        
        # update distance to objcet in front of the robot
        self.distance = data.ranges[0]



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
