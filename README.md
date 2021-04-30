# q_learning_project

## Implementation Planning

1. The names of your team members:
Dante Gil-Marin, Yoon Jeong

2. A 1-2 sentence description of how your team plans to implement each of the following components of this project as well as a 1-2 sentence description of how you will test each component:

Q-learning algorithm
- Executing the Q-learning algorithm
**Implementation**: We will have a while loop executing the algorithm outlined in the write-up. Selection of a_t at random will be done using the python random library. We will perform the a_t by altering the action matrix. We will receive the reward by using the state matrix.
**Testing**: We will do a single iteration of the matrix by hand and do an eyeball check after using a print statement. We will test these portions by using an `if` statement rather than a `while` statement and print the results to see if a single time step executes correctly. 

- Determining when the Q-matrix has converged
**Implementation**: We can do an equality check and store the last state of the Q-matrix at the previous time step.
**Testing**: We will try to iterate through a simple Q-matrix by hand and see that the number of time steps taken is equal in the algorithm implementation as it is when we do it by hand.

- Once the Q-matrix has converged, how to determine which actions the robot should take to maximize expected reward
**Implementation**: When it converges, we figure out the current state and we use the Q matrix to find that state. And then the column of the Q matrix represents the next action to take us to the appropriate state.
**Testing**: Similarly to the above, we will test this by using our Q-matrix solved by hand and checking to see if the action the robot chooses is the same as the one determined by hand.


Robot perception
- Determining the identities and locations of the three colored dumbbells
**Implementation**:  We will use color recognition using OpenCV using the range of hues available in the HSV swatches.
**Testing**: We can use RViz to observe if the identified dumbbells match up with the camera's vision. We will also use print statements to ensure that the identification has happened correctly.

- Determining the identities and locations of the three numbered blocks
**Implementation**: We will use this github repo https://github.com/jswon/tensorflow_ros_mnist or keras-ocr to implement tensorflow trained on MNIST to recognize digits in realtime off the robot RGB camera.
**Testing**: We will have the robot print out what digit it is seeing when it recognizes something. Then we will show it different labeled blocks making sure it identifies them correctly.

Robot manipulation & movement
- Picking up and putting down the dumbbells with the OpenMANIPULATOR arm
**Implementation**: We will use the `/q_learning/robot_action` rospy subscription to move the robot arm.
**Testing**: We will test by visually inspecting the robot in the simulator and seeing whether it moves according to the basic instructions we give it.

- Navigating to the appropriate locations to pick up and put down the dumbbells
**Implementation**: We will first identify where the numbered block is using the above perception step, and then move to the desired location using the `RobotMoveDBToBlock` subscription message.
**Testing**: We will be able to tell when the robot has correctly identified where to go to by visually inspecting in the simulator.

3. A brief timeline sketching out when you would like to have accomplished each of the components listed above.
By next Wednesday our plan is to have completed at minimum the Q learning matrix algorithm and robot movement. We hope that by next Friday we will have implemented the other portions. We hope to be finished with the project by the Sunday prior to the project being due, and just be working on debugging if necessary on the Monday and Tuesday afterwards.


