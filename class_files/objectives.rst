Objectives
==========

The objectives for this module are to:

* Understand the fundamental principles of robot sensing using reflectance sensors.
* Learn to program a robot to perceive and react to its environment based on sensor data.
* Develop skills in implementing different control strategies for robot navigation, including conditional logic and proportional control.
* Gain practical experience in structuring robot code for reusability and maintainability through functions and object-oriented programming.
* Achieve the capability to program a robot to autonomously follow a line and navigate a defined track.

Sensing Lines
-------------

* Understand how the XRP robot's reflectance sensors work to detect lines.
* Learn how to read values from the left and right reflectance sensors using **XRPLib**.
* Characterize the reflectance sensors by observing the values they return over different surfaces (white, black line, partially over the line).
* Determine an appropriate threshold value to differentiate between the line and the background.
* Learn about conditional statements (`if`/`else`) in Python and how they can be used to make decisions based on sensor input.
* Write a program that makes the XRP robot drive forward and stop when both reflectance sensors detect a line.

Staying Inside a Circle
-----------------------

* Understand the concept and benefits of using functions in Python for code organization and reuse.
* Learn how to define a function with parameters and a return value.
* Write a function called ``is_over_line()`` that checks if both of the robot's reflectance sensors are over a line using a specified threshold.
* Learn how to import and use the `random` library in Python to generate random numbers.
* Program the XRP robot to stay inside a circular boundary by detecting the line, backing up, and turning at a random angle.

Basketball Drills
-----------------

* Understand the concept of classes in Python and how they help organize code.
* Learn about the different parts of a class: attributes, methods, and the constructor (`__init__`).
* Implement a `LineSensor` class to encapsulate the functionality of reading and interpreting data from the robot's reflectance sensors.
* Implement a `LineTracker` class that utilizes the `LineSensor` to perform line-following actions, such as driving until a line is detected.
* Understand how to use lists in Python to store sequences of data.
* Program the XRP robot to perform a "pacer" basketball drill by driving to specified distances and returning to a line.

Following Lines: On-Off Control
-------------------------------

* Understand the limitations of odometry (estimating position based on wheel turns) for precise navigation.
* Learn how reflectance sensors can be used for line following as a more accurate sensor-based navigation method.
* Understand the basic logic of on-off control for line following: going straight when centered, turning right when veering left, and turning left when veering right.
* Learn how to calculate the error between left and right sensor readings to determine the robot's position relative to the line.
* Modify the ``LineSensor`` class to include a ``get_error`` function that calculates the difference between left and right sensor readings.
* Modify the ``LineTracker`` class to include a ``line_follow_on_off`` function that implements on-off control for line following.
* Program the XRP robot to follow a line using the on-off control method.

Following the Line: Proportional Control
----------------------------------------

* Understand the concept of proportional control in robotics and how it differs from on-off control.
* Learn how to implement proportional control for line following by adjusting motor speeds based on the error between sensor readings.
* Understand the role of the proportional gain (KP) and how it affects the robot's line following behavior.
* Learn how to tune the KP value through experimentation to achieve smooth and stable line following.
* Implement a "racing around a circle" activity using proportional control and the line sensor to detect an intersection and turn around.

Final Project
-------------

The final project for this module will be to have the robot drive around drawn
cicle a number of times, turning around at the intersection and continuing in
the opposite direction. You will compete with your classmates to see who's
robot can complete the challlenge in the fastest time.