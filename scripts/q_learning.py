#!/usr/bin/env python3

import rospy
import numpy as np

class QLearning(object):
    def __init__(self):
        # Initialize this node
        rospy.init_node("q_learning")

        # Fetch pre-built action matrix
        self.action_matrix = np.loadtxt("../action_matrix.txt")


        # Fetch actions. These are the only 9 possible actions the system can take.
        # self.actions is an array of dictionaries where the row index corresponds
        # to the action number, and the value has the following form:
        # { dumbbell: "red", block: 1}
        colors = ["red", "green", "blue"]
        self.actions = np.loadtxt("../actions.txt")
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
        self.states = np.loadtxt("../states.txt")
        self.states = list(map(lambda x: list(map(lambda y: int(y), x)), self.states))

    def save_q_matrix(self):
        # TODO
        return
if __name__ == "__main__":
    node = QLearning()
