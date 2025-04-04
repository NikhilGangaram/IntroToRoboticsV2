Basketball Drills
=================

**Daily Goals**

* Understand the concept of classes in Python and how they help organize code.
* Learn about the different parts of a class: attributes, methods, and the constructor (`__init__`).
* Implement a `LineSensor` class to encapsulate the functionality of reading and interpreting data from the robot's reflectance sensors.
* Implement a `LineTracker` class that utilizes the `LineSensor` to perform line-following actions, such as driving until a line is detected.
* Understand how to use lists in Python to store sequences of data.
* Program the XRP robot to perform a "pacer" basketball drill by driving to specified distances and returning to a line.

Introduction
------------

Today, we're going to make the XRP perform basketball drills! We'll use the reflectance sensors to detect lines on the floor and drive the robot to different distances. To keep our code organized and reusable, we'll use **classes** to structure our program.

Think of a **class** like a blueprint for building something, like a house. The blueprint tells you all the parts the house will have (like rooms, doors, and windows) and what you can do with it (like open the door or turn on the lights). In programming, a class is a blueprint for creating objects.

What is a Class?
----------------

In Python, a class is a blueprint for creating objects. An object is an instance of a class, meaning it follows the structure and behavior defined by the class. Using classes allows us to group related data and functions together, making our program more modular and easier to manage.

Instead of having separate variables for sensor values and multiple functions scattered throughout our code, we can encapsulate them within a class. This makes it easier to maintain and extend our program.

Parts of a Class
----------------

A class consists of:

* **Attributes (Fields):** These are like the characteristics or features of your robot. For example, the speed of the motors or the readings from the sensors. They are variables that store information about an instance of the class.
* **Methods:** These are the actions your robot can perform. For example, moving forward, turning, or checking if it's over a line. They are functions that define the behavior of the instance.
* **Constructor (\_\_init\_\_ method):** This is a special set of instructions that runs automatically when you create a new robot based on the plan. It's like assembling the basic parts of your toy robot. It's a special method that initializes an instance when it is created.

Example: A Simple Student Class
-------------------------------

Here's a basic example of how a class works in Python:

.. code-block:: python

   class Student:
       def __init__(self, name, age, major):
           """Initialize the student with a name, age, and major."""
           self.name = name  # Attribute to store the student's name
           self.age = age  # Attribute to store the student's age
           self.major = major  # Attribute to store the student's major

       def introduce(self):
           """Introduce the student."""
           print(f"Hi, I'm {self.name}, a {self.age}-year-old {self.major} major.")

   # Creating instances of the Student class
   student1 = Student("Alice", 20, "Computer Science")
   student2 = Student("Bob", 22, "Mechanical Engineering")

   # Calling methods on the objects
   student1.introduce()
   student2.introduce()

In this example, `Student` is our blueprint. When we write `student1 = Student("Alice", 20, "Computer Science")`, we are actually building a new student object named `student1` based on the `Student` blueprint and giving it specific information (name, age, major).

Understanding self
------------------

In the Student class, you might have noticed the use of ``self``. Let's break down what it means and why it's important.

Think of ``self`` as the name tag for the specific robot (or student in the example) you are currently working with. ``self`` is a reference to the current instance of the class. It allows us to access the attributes and methods of the class in Python.

When we define a method in a class, we always include ``self`` as the first parameter. This is how Python knows that the method belongs to the instance.

For example, in the ``introduce`` method, ``self.name``, ``self.age``, and ``self.major`` refer to the specific instance's attributes. When we create multiple students, each instance maintains its own unique data.

Using functionality from our previous lessons, let's create two classes: ``LineSensor`` and ``LineTracker``. This will help us organize our robot's code for the basketball drills.

``LineSensor`` will directly interact with the reflectance sensors to determine if the robot is over a line.
``LineTracker`` will use a ``LineSensor`` object to manage line-tracking functionality and provide higher-level operations.

Class Implementation
--------------------

Let's first implement the ``LineSensor`` class. Our first blueprint is for the `LineSensor`. Its job is simple: to talk directly to the robot's reflectance sensors and tell us if they see a line. As a reminder, the ``LineSensor`` class will solely return information directly from the reflectance sensors.

.. code-block:: python

   from XRPLib.defaults import *

   class LineSensor:
       def __init__(self):
           """Initializes the line sensor by setting up the reflectance sensors."""
           self.left_sensor = reflectance.get_left
           self.right_sensor = reflectance.get_right

       def is_over_line(self, threshold):
           """Checks if either sensor is over the line."""
           left_over_line = self.left_sensor() > threshold
           right_over_line = self.right_sensor() > threshold
           return left_over_line and right_over_line

       def report_values(self):
           left = self.left_sensor()
           right = self.right_sensor()
           print(f'left: {left}, right: {right}')

This `LineSensor` class has a few important parts:

* ``__init__``: This sets up the sensor. It gets the functions that read the left and right reflectance sensors (`reflectance.get_left` and `reflectance.get_right`) and stores them as `self.left_sensor` and `self.right_sensor` so we can use them later.
* ``is_over_line``: This method takes a ``threshold`` (a value that determines what counts as a line) as input. It checks if the value from *both* the left *and* the right sensor is greater than this threshold. If both are, it returns `True`, meaning the robot is likely over the line. Otherwise, it returns `False`. *(Note: Depending on your robot's sensor setup and the line width, you might want to change `and` to `or` here initially, so it detects a line if *either* sensor sees it. You can adjust this later.)*
* ``report_values``: This is a helpful method to see the raw values from the left and right sensors. It reads the current values and prints them to the console, which can be useful for debugging or setting the correct threshold.

Defining the ``LineSensor`` class allows us to encapsulate the sensor functionality and reuse it in other parts of our code. Furthermore, it provides a modular foundation which can be easily modified or replicated in the future in case we would want to use a different type of line sensor. Now, let's create the ``LineTracker`` class that uses the ``LineSensor`` class to track lines.

.. code-block:: python

   from XRPLib.defaults import *

   class LineTracker:
       def __init__(self, drivetrain):
           """Initializes the line tracker with a LineSensor object and a drivetrain object."""
           self.sensor = LineSensor()
           self.drivetrain = drivetrain

       def drive_until_line(self, threshold, speed):
           """Drives forward until the line sensors detect the robot is over the line."""
           self.drivetrain.set_speed(speed, speed)
           while not self.sensor.is_over_line(threshold):
               pass  # Keep driving until the line is detected
           self.drivetrain.stop()

The ``LineTracker`` class builds upon the ``LineSensor`` and helps the robot perform a more complex action: driving until it sees a line. Let's look at its parts:

* ``__init__``: When we create a `LineTracker`, we need to give it a ``drivetrain`` object. The ``drivetrain`` (which we assume is defined in `XRPLib.defaults`) allows us to control the robot's motors. Inside the `__init__` method, we also create an instance of our `LineSensor` class and store it as `self.sensor`. This means each `LineTracker` will have its own `LineSensor` to work with.
* ``drive_until_line``: This method takes a ``threshold`` (for the line sensor) and a ``speed`` as input. It first sets the robot's motors to the given speed to drive forward. Then, it enters a ``while not`` loop. This loop will continue to run *as long as* the condition inside it is `True`. In this case, the condition is `self.sensor.is_over_line(threshold)`. The `not` in front means the loop continues as long as the sensor is *not* over the line. Inside the loop, `pass` means the robot just keeps doing what it was doing (driving forward). Once the `self.sensor.is_over_line(threshold)` becomes `True` (meaning the robot has detected a line), the loop stops, and the next line of code, `self.drivetrain.stop()`, is executed, making the robot stop.

Driving Until the Line
----------------------

Now that we have our ``LineTracker`` class, let's use it to drive the robot forward until it detects a line.

.. code-block:: python

   from XRPLib.defaults import *

   tracker = LineTracker(drivetrain)
   tracker.drive_until_line(0.5, 5)

Here, we are creating an instance (a specific robot controller) of our `LineTracker` class, giving it the robot's `drivetrain`. Then, we tell it to execute the `drive_until_line` method with a threshold of `0.5` and a speed of `5`. This will make the robot drive forward until the reflectance sensors detect a line (where the sensor readings are greater than 0.5), and then it will stop.

.. figure:: images/stop_at_line.webp
   :width: 450

.. note::

   When refactoring code, it's always beneficial to ensure that previous functionality is preserved. This ensures that we haven't lost any functionality in our code, and now, it's just written in a better, more maintainable way. Refactoring should improve the structure and readability of the code without altering its external behavior.

Introduction to Lists
---------------------

Imagine you have a shopping list with different items you need to buy. In Python, a **list** is like that shopping list – it's a way to store multiple pieces of information in one place, in a specific order.

Example of a list:

.. code-block:: python

    distances = [10, 20, 15, 25]  # Distances in some unit

We can use a `for` loop to iterate through each item (in this case, each distance) in the list.

Example:

.. code-block:: python

    for distance in distances:
        print("Traveling", distance, "units")

To access a specific item in a list, we use square brackets ``[]`` with the index number. Note that Python uses zero-based indexing, so the first element is at index 0.

Example:

.. code-block:: python

    first_distance = distances[0]  # Access the first element
    print("First distance:", first_distance)

    last_distance = distances[-1]  # Access the last element
    print("Last distance:", last_distance)

Basketball Drill: Pacers
------------------------

In basketball, a pacer drill involves running to a series of increasing distances, turning around, and returning to the starting line. We will program the robot to perform a similar drill.

Code Implementation
-------------------

.. code-block:: python

    from XRPLib.defaults import *

    tracker = LineTracker(drivetrain)  # Create a LineTracker object
    distances = [10, 20, 15, 25]  # Define the list of distances

    for distance in distances:  # Iterate through each distance
        drivetrain.set_speed(5, 5)  # Drive forward
        drivetrain.drive_distance(distance)  # Travel the given distance
        drivetrain.turn_degrees(180)  # Turn around

        tracker.drive_until_line(0.5, 5)  # Drive back to start using LineTracker

        print(f"Completed drill for {distance} units.")

Let's break down this code for the pacer drill:

* ``tracker = LineTracker(drivetrain)``: We create our line tracker object again, giving it control of the robot's wheels.
* ``distances = [10, 20, 15, 25]``: This is our list of distances that the robot will travel in each step of the drill.
* ``for distance in distances:``: This starts a loop that will go through each distance in our `distances` list one by one. For each distance in the list, the code inside the loop will be executed.
* ``drivetrain.set_speed(5, 5)``: Sets the robot's speed for moving forward.
* ``drivetrain.drive_distance(distance)``: This command (which we assume is defined elsewhere in the `XRPLib`) makes the robot drive forward by the current `distance` from our list.
* ``drivetrain.turn_degrees(180)``: Tells the robot to turn around.
* ``tracker.drive_until_line(0.5, 5)``: Uses our `LineTracker` to drive the robot back to the starting line (where we assume there's a line).
* ``print(f"Completed drill for {distance} units.")``: Prints a message to the console indicating the completion of that step of the drill, showing the distance it just traveled.

Try It Out
----------

Run the program on the robot.
Observe how it travels to different distances, turns around, and stops at the line.
Modify the list of distances and see how it changes the robot's movement.

By using classes like ``LineSensor`` and ``LineTracker``, we've made our code more organized and easier to understand. Each class has a specific job, making it simpler to manage and modify our robot's behavior for the basketball drills.

.. error::

    add a video

**Recap**

Today, you have:

* Learned about the fundamental concept of classes in Python and how they help in organizing and structuring code into reusable components.
* Understood the key components of a class, including attributes (data), methods (functions within a class), and the constructor (`__init__`) for initializing objects.
* Implemented a `LineSensor` class to handle the direct interaction with the XRP robot's reflectance sensors, including reading sensor values and determining if a line is detected.
* Implemented a `LineTracker` class that utilizes the `LineSensor` object to perform a higher-level action: driving the robot until a line is detected.
* Learned how to use lists in Python to store a collection of data, specifically a sequence of distances for the basketball drill.
* Programmed the XRP robot to perform a "pacer" basketball drill, which involves driving to a series of predefined distances and then using the `LineTracker` to return to a starting line.