#!/usr/bin/env python3

import rospy
import numpy as np
import os
from time import sleep

import random
from q_learning_project.msg import QMatrix, QMatrixRow, QLearningReward, RobotMoveDBToBlock

# Path of directory on where this file is located
path_prefix = os.path.dirname(__file__) + "/action_states/"

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        print("Starting to set up publishers")
        # Setup publishers and subscribers
        self.q_pub = rospy.Publisher("/q_learning/q_matrix", QMatrix, queue_size=10)
        self.action_pub = rospy.Publisher("/q_learning/robot_action", RobotMoveDBToBlock, queue_size=1)
        rospy.Subscriber("/q_learning/reward", QLearningReward, self.receive_reward)
        print("Finished setting up publishers")
        # Give some time to initialize
        
        # Fetch pre-built action matrix. This is a 2d numpy array where row indexes
        # correspond to the starting state and column indexes are the next states.
        #
        # A value of -1 indicates that it is not possible to get to the next state
        # from the starting state. Values 0-9 correspond to what action is needed
        # to go to the next state.
        #
        # e.g. self.action_matrix[0][12] = 5
        self.action_matrix = np.loadtxt(path_prefix + "action_matrix.txt")

        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { dumbbell: "red", block: 1}
        colors = ["red", "green", "blue"]
        self.actions = np.loadtxt(path_prefix + "actions.txt")
        self.actions = list(map(
            lambda x: {"dumbbell": colors[int(x[0])], "block": int(x[1])},
            self.actions
        ))


        # Fetch states. There are 64 states. Each row index corresponds to the
        # state number, and the value is a list of 3 items indicating the positions
        # of the red, green, blue dumbbells respectively.
        # e.g. [[0, 0, 0], [1, 0 , 0], [2, 0, 0], ..., [3, 3, 3]]
        # e.g. [0, 1, 2] indicates that the green dumbbell is at block 1, and blue at block 2.
        # A value of 0 corresponds to the origin. 1/2/3 corresponds to the block number.
        # Note: that not all states are possible to get to.
        self.states = np.loadtxt(path_prefix + "states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

        print("About to init q table")
        # initialize q table
        """64 rows = number of states
        9 cols = number of actions"""
        self.q_matrix = np.zeros((64, 9))
        # initialize extraneous algorithm values
        self.history = []
        self.time = 0

        # save states
        self.action_queue = []
        self.curr_state = 0
        self.go = False
        sleep(2)
        print("Finishing init")



    def has_converged(self):
        """ Checks to see if the Q-Matrix has converged
        or in other words, if the last 100 matrices are the same.
        """
        def all_equal(arr):
            for i in arr:
                if not np.array_equal(i, arr[0]):
                    return False
            return True

        hist_limit = 100
        if len(self.history) < 1000:
            return False
        
        # checks to see if the last `hist_limit` q-matrices are the same
        convergence = all_equal(self.history[-hist_limit:])
        return convergence

    def perform_action(self, act, next_state):
        """ Pushes action to action publisher
            Pushes an action to self.action_queue
        """
        # translate action into message
        action = self.actions[act]
        dumbbell, block = action["dumbbell"], action["block"]

        self.go = False
        # queue up the action
        self.action_queue.append((act, next_state))

        # perform action
        self.action_pub.publish(dumbbell, block)

    def get_valid_actions(self, state: int) -> list:
        """ Gets list of all possible actions 
        """
        action_row = self.action_matrix[state]
        valid_actions = []
        for next_state, action in enumerate(action_row):
            if 0 <= action <= 8:
                valid_actions.append((int(action), next_state))
        return valid_actions

    def receive_reward(self, data):
        """
        Runs after perform_action has been executed.
        Runs the remainder of the Q-Learning algorithm
        """
        if len(self.action_queue) == 0:
            print("Received reward but no action in queue.")
            return

        action, next_state = self.action_queue.pop(0)
        # calculate Q(s_t, a_t) + alpha * (r_t + gamma * max_a Q(s_t+1, a) - Q(s_t, a_t))
        print(f"Got reward of {data.reward}. Iteration: {self.time}")

        # adjustable vars
        gamma = 0.8
        alpha = 1.0

        # calculate
        if len(self.get_valid_actions(self.curr_state)) == 0: 
            max_q = 0
        else:
            max_q = max(self.q_matrix[next_state])

        curr_q = self.q_matrix[self.curr_state][action]
        to_set = curr_q + alpha * (data.reward + (gamma * max_q) - curr_q)
        self.q_matrix[self.curr_state][action] = to_set

        # update state
        self.history.append(self.q_matrix)
        self.time += 1
        if len(self.get_valid_actions(self.curr_state)) == 0: 
            self.curr_state = 0
        else:
            self.curr_state = next_state

        self.go = True
        self.publish_q_matrix()

    def publish_q_matrix(self):
        """ Converts self.q_matrix (a numpy array)
            into the form of a QMatrix object
        """
        matrix = QMatrix()
        matrix.q_matrix = []
        for q_row in self.q_matrix:
            row = QMatrixRow()
            row.q_matrix_row = q_row.tolist()
            matrix.q_matrix.append(row)
        self.q_pub.publish(matrix)

        filename = os.path.dirname(__file__) + "/saved_matrix/q_matrix.txt"
        np.savetxt(filename, self.q_matrix, delimiter=" ")


    def load_in_actions_and_matrix(self):
        q_matrix = np.loadtxt(os.path.dirname(__file__) + "/saved_matrix/q_matrix.txt", delimiter=" ")
        action_matrix = np.loadtxt(os.path.dirname(__file__) + "/action_states/action_matrix.txt")
        colors = ["RED", "GREEN", "BLUE"]
        actions = np.loadtxt(os.path.dirname(__file__) + "/action_states/actions.txt")
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

        out = [(first_db, first_block), (second_db, second_block), (third_db, third_block)]        

    def run(self):
        print("Now entering run function")
        # send the first action
        # rospy.sleep(0.5)
        valid_actions = self.get_valid_actions(self.curr_state)
        a_t, next_state = random.choice(valid_actions)
        self.perform_action(a_t, next_state)
        # rospy.sleep(0.5)

        print("Starting the loop")
        print(f"{self.go}")
        # loop q-learning algo
        while not self.has_converged():
            if self.go:
                valid_actions = self.get_valid_actions(self.curr_state)

                if valid_actions == []:
                    self.curr_state = 0
                    valid_actions = self.get_valid_actions(0)
                a_t, next_state = random.choice(valid_actions)
                self.perform_action(a_t, next_state)
            else:
                continue
            # choose action
            # print(f"There were {len(valid_actions)} valid actions.")
            # print(f"Chose action {a_t} from state {self.curr_state}.")
            # perform a_t, triggering rest of algo
            # print(f"Performing action {a_t}")
            # give the action a bit of time

            
        print("Finished training QMatrix.")


if __name__ == "__main__":
    print("Now starting q_learning")
    node = QLearning()
    print("About to run")
    node.run()