# q_learning_project

## YouTube
We were having difficulty converting the gif using ffmpeg so we uploaded the final mp4 file to Youtube.
You can watch it here:
https://youtu.be/4yq8lTnCyd0

## GIF
![gif][gif]

## Writeup

### Objectives Description
The goal of this project is to implement a robot that uses the reinforcement learning algorithm known as Q-learning to correctly place a set of colored dumbbells in front of numbered cubes. First, the robot is trained repeatedly on a system that rewards it when it correctly guesses a given permutation of randomly placed colored dumbbells and cubes. This slowly fills the Q-matrix, using the Q-learning algorithm. The robot then uses the information from the final Q-matrix to move the colored dumbbells to the correct numbered cubes.

### High-Level Description
We determined which dumbbells belong in front of each numbered cube by using the reinforcement learning algorithm known as Q-learning. This was done by generating a Q-matrix where the rows represent the states the robot can be in and the columns represent the actions the robot can take. In order to reach the optimal Q-matrix we picked a random set of actions to walk through. The correct set of actions would give us a reward, which we used to update our Q-matrix according to the formula given by the Q-learning algorithm. We repeated these steps until the Q-matrix did not change for 100 iterations, indicating convergence. Once we had the optimal values for each given state, we could walk through the maximal reward values to get a "map" of what the optimal path was. This optimal path gave us the colored dumbbell that corresponds to each block — and we used this information to move the robot to the appropriate block.

### Q-Learning Algorithm Description

#### 1. Selecting and executing actions for the robot (or phantom robot) to take
For this, we were aided by a matrix provided to us containing all possible actions for any given state. We simply iterated through this matrix in order to determine what actions were possible and then used the random library to randomly select an action from there.

#### 2. Updating the Q-matrix
For updating the Q-matrix, this was as simple as repeating the equation given to us in the algorithm. The values of alpha and gamma were 1.0 and 0.8, respectively.

#### 3. Determining when to stop iterating through the Q-learning algorithm
We stop iterating through the Q-learning algorithm once the matrix does not change after at least 100 iterations. The way we check for this is by saving the matrix to a `self.history` variable and checking the last 100 matrices to see if they are identical.

#### 4. Executing the path most likely to lead to receiving a reward after the Q-matrix has converged on the simulated Turtlebot3 robot
For solving this, we simply start at a state of 0, then follow the action that has the highest reward on the Q-matrix. In the cases where the rewards are identical, we simply choose the first instance. By the end of this process, there should be three different dumbbells, each assigned to three different blocks.

### Robot Manipulation and Movement

Moving to the right spot in order to pick up a dumbbell was accomplished in `execute_phase_0`. The robot uses the RGB camera subscriber to call the function which uses logic similar to that of the line follower bot to move toward the correct colored dumbbell.

Picking up the dumbbell was accomplished in `execute_phase_1`. The robot uses the RGB camera subscriber to detect the distance to the dumbbell and then uses inverse kinematics to pick up the dumbbell.

Moving to the desired destination (numbered block) with the dumbbell was accomplished in `execute_phase_2`. The robot uses the RGB camera subscriber for two tasks. First it detects the digit (color black) and centers it in the camera. Then it takes that image and runs it through the OCR pipeline producing checking for the goal digit. If it doesn’t find it it moves to the next digit. When it finally finds the correct digit it drives forward and stops when at an appropriate distance.

Putting the dumbbell back down at the desired location is accomplished in `execute_phase_3`. This is done in the same way as picking the dumbbell up but in reverse.

In `execute_phase_4` the action queue is popped and we move with the next objective back to `execute_phase_0`.


### Challenges
We faced a number of challenges as a team. Issues with NoMachine were a given — the virtual machine would sometimes freeze, rendering itself inaccessible and making the robotics simulation untestable. We overcame this as a team by using the LiveShare feature on VSCode — both individuals could write code, even if only one partner could actually test it.
We had problems with using the simple function `rospy.sleep()`, which inexplicably caused the program to freeze when we used it. Our workaround was simply to use the `time.sleep()` function instead.
Our largest problem perhaps was with the `keras` and `tensorflow` libraries, which inexplicably had version control issues. We resolved these by using `conda` (Dante's familiarity with this piece of software was invaluable!) to set our version of Python to 3.6 so that we could use an older version of `tensorflow` (Google revamped Tensorflow in version 2.0, which is only supported by Python 3.8+).


### Future Work
In the future, we would like to optimize for image quality by driving up to the block before we take the image. We would also like to make the robot’s actions independent of the room it is working in since currently some of the actions are timed to fit the geometry of the arena. Finally, we would like to refactor the code so that it does not run from the image subscriber callback. 

### Takeaways
* Parallelizing the tasks 

Though the project was clearly presented in a modular fashion, it was not until late into the project (after we were nearly done with the Q-learning algorithm) that we truly began to parallelizing the tasks of picking up/dropping dumbbells, photographic capture of the blocks, and moving from one place to another. If we were to do this project again, one of us would immediately have begun to work on kinematics while the other worked on Q-learning.

* Figuring out workarounds to asynchronous Python

Both me and Dante have experience using asynchronous/callback functions in JavaScript, but were not familiar with conventions in Python. For this reason, we often had to use "creative" workarounds such as setting an accessory variable such as `self.phase = -1`, or `self.go = False` inside of a while loop so that the loop would do nothing while another task executed properly. When we were finished with executing a task and could continue, we would change the value.

## Implementation Planning

1. The names of your team members:

Dante Gil-Marin, Yoon Jeong

2. A 1-2 sentence description of how your team plans to implement each of the following components of this project as well as a 1-2 sentence description of how you will test each component:

### Q-learning algorithm
- Executing the Q-learning algorithm

**Implementation**: We will have a while loop executing the algorithm outlined in the write-up. Selection of a_t at random will be done using the python random library. We will perform the a_t by altering the action matrix. We will receive the reward by using the state matrix.

**Testing**: We will do a single iteration of the matrix by hand and do an eyeball check after using a print statement. We will test these portions by using an `if` statement rather than a `while` statement and print the results to see if a single time step executes correctly. 

- Determining when the Q-matrix has converged

**Implementation**: We can do an equality check and store the last state of the Q-matrix at the previous time step.

**Testing**: We will try to iterate through a simple Q-matrix by hand and see that the number of time steps taken is equal in the algorithm implementation as it is when we do it by hand.

- Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward

**Implementation**: When it converges, we figure out the current state and we use the Q matrix to find that state. And then the column of the Q matrix represents the next action to take us to the appropriate state.

**Testing**: Similarly to the above, we will test this by using our Q-matrix solved by hand and checking to see if the action the robot chooses is the same as the one determined by hand.


### Robot perception
- Determining the identities and locations of the three colored dumbbells

**Implementation**:  We will use color recognition using OpenCV using the range of hues available in the HSV swatches.

**Testing**: We can use RViz to observe if the identified dumbbells match up with the camera's vision. We will also use print statements to ensure that the identification has happened correctly.

- Determining the identities and locations of the three numbered blocks

**Implementation**: We will use this github repo https://github.com/jswon/tensorflow_ros_mnist or keras-ocr to implement tensorflow trained on MNIST to recognize digits in realtime off the robot RGB camera.

**Testing**: We will have the robot print out what digit it is seeing when it recognizes something. Then we will show it different labeled blocks making sure it identifies them correctly.

### Robot manipulation & movement
- Picking up and putting down the dumbbells with the OpenMANIPULATOR arm

**Implementation**: We will use the `/q_learning/robot_action` rospy subscription to move the robot arm.

**Testing**: We will test by visually inspecting the robot in the simulator and seeing whether it moves according to the basic instructions we give it.

- Navigating to the appropriate locations to pick up and put down the dumbbells

**Implementation**: We will first identify where the numbered block is using the above perception step, and then move to the desired location using the `RobotMoveDBToBlock` subscription message.

**Testing**: We will be able to tell when the robot has correctly identified where to go to by visually inspecting in the simulator.

3. A brief timeline sketching out when you would like to have accomplished each of the components listed above.

By next Wednesday our plan is to have completed at minimum the Q learning matrix algorithm and robot movement. We hope that by next Friday we will have implemented the other portions. We hope to be finished with the project by the Sunday prior to the project being due, and just be working on debugging if necessary on the Monday and Tuesday afterwards.

[gif]: ./media/success.gif
