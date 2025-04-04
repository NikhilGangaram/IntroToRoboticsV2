Staying Inside a Circle
=======================

**Daily Goals**

* Understand the concept and benefits of using functions in Python for code organization and reuse.
* Learn how to define a function with parameters and a return value.
* Write a function called ``is_over_line()`` that checks if both of the robot's reflectance sensors are over a line using a specified threshold.
* Learn how to import and use the `random` library in Python to generate random numbers.
* Program the XRP robot to stay inside a circular boundary by detecting the line, backing up, and turning at a random angle.

Introduction
------------

In this lesson, you will program the XRP to stay inside a circle by using the line-sensing skills you developed in the previous lesson. This time, however, we'll learn how to package code into reusable blocks called **functions**. Functions will make our program more organized and easier to manage, especially when we want to perform the same set of actions multiple times.

We'll then use our function to ensure that the robot stays inside a circular boundary drawn on the floor. The robot will drive forward until its sensors detect the line of the circle. When it sees the line, it will back up, turn at a random angle, and then continue driving forward, trying to stay within the circle.

Functions in Python
-------------------

Imagine you have a set of instructions that you want to use over and over again in your program, like a mini-program within your main program. In Python, we can create these mini-programs called **functions**.

Think of a function like a recipe. You give it some ingredients (called **parameters**), it follows a set of steps (the code inside the function), and then it gives you a final dish (the **return value**).

Functions are a fundamental concept in programming and provide several benefits:

- **Code Reuse**: Once you write a function, you can use it as many times as you need without writing the same code again. This saves you time and effort!
- **Separation of Responsibilities**: Functions help you break down a big, complicated task into smaller, more manageable parts. Each function can handle a specific job, making your program easier to think about and understand.
- **Improved Readability**: By giving a block of code a meaningful name (the function name), you make your program easier to read and understand. It's like giving a title to a chapter in a book.

A function typically has the following parts:

- **Function Name**: A unique name that identifies the function. You'll use this name to "call" or run the function in your program.
- **Parameters**: These are like the ingredients you give to the recipe. Parameters are values that you can pass into the function when you call it. These values can customize how the function behaves. A function can have no parameters, one parameter, or many parameters.
- **Return Value**: This is the result that the function produces after it finishes running. It's like the dish the recipe makes. The function can send this result back to the part of the program that called it.

Here is an example of a simple function in Python:

.. code-block:: python

    def greet(name):
        return f"Hello, {name}!"

    print(greet("Alice"))  # Output: Hello, Alice!

In this example:

* `def greet(name):` defines a function named `greet`. It takes one parameter called `name`.
* `return f"Hello, {name}!"` is the code inside the function. It creates a greeting message using the provided `name` and sends it back as the result of the function.
* `print(greet("Alice"))` calls the `greet` function, passing the value "Alice" as the `name`. The function returns "Hello, Alice!", which is then printed to the screen.

Writing the ``is_over_line`` Function
--------------------------------------

Now, let's create our own function that will tell us if the XRP robot's sensors are over a dark line. We'll call this function ``is_over_line()``. This function will take one **parameter**, which will be the threshold value we determined in the previous lesson. Remember, the threshold helps us decide what sensor reading counts as "seeing a line."

The ``is_over_line()`` function will **return** either ``True`` (if both sensors detect a line) or ``False`` (if at least one sensor does not detect a line).

.. note::

    Think back to the "Stopping at a Line" exercise in the previous lesson. You wrote code to check if both the left and right sensors were detecting the line. You can use the same logic inside our new ``is_over_line()`` function.

Here's one way you can write the ``is_over_line()`` function in Python:

.. code-block:: python

    from XRPLib.defaults import *

    def is_over_line(threshold):
        # Get the current value from the left reflectance sensor
        left_sensor_value = reflectance.get_left()
        # Check if the left sensor value is greater than our threshold
        if left_sensor_value > threshold:
            left_over_line = True  # If it is, we think it's over the line
        else:
            left_over_line = False # Otherwise, it's not

        # Do the same for the right reflectance sensor
        right_sensor_value = reflectance.get_right()
        if right_sensor_value > threshold:
            right_over_line = True
        else:
            right_over_line = False

        # The function will return True only if *both* left_over_line and right_over_line are True
        return left_over_line and right_over_line

In this code:

* `def is_over_line(threshold):` defines a function named `is_over_line` that takes one parameter called `threshold`.
* Inside the function, we first get the current readings from the left and right reflectance sensors using `reflectance.get_left()` and `reflectance.get_right()`.
* For each sensor, we use an `if` statement to check if its value is greater than the `threshold`. If it is, we set a variable (`left_over_line` or `right_over_line`) to `True`. Otherwise, we set it to `False`.
* Finally, the `return left_over_line and right_over_line` line means the function will only send back a value of `True` if both `left_over_line` and `right_over_line` are `True` (meaning both sensors detected the line). If either one is `False`, the function will return `False`.

.. note::

    Just like before, you can write the ``is_over_line()`` function in a more concise way using a single `return` statement:

    .. code-block:: python

        from XRPLib.defaults import *

        def is_over_line(threshold):
            return reflectance.get_left() > threshold and reflectance.get_right() > threshold

    While this version does the exact same thing, we wrote the first version with the `if/else` statements to make it easier for you to understand each step.

Next, let's see how we can use this new `is_over_line()` function in our program to make the robot drive forward until it detects a line.

Here's how you might modify your code from the previous lesson:

.. code-block:: python

    from XRPLib.defaults import *

    def is_over_line(threshold):
        # Check the left sensor
        if reflectance.get_left() > threshold:
            left_over_line = True
        else:
            left_over_line = False

        # Check the right sensor
        if reflectance.get_right() > threshold:
            right_over_line = True
        else:
            right_over_line = False

        # Return True if both sensors are over the line
        return left_over_line and right_over_line

    line_threshold = 0.5  # Remember to use the threshold value you determined!

    # Set the speed of both wheels to make the robot move forward slowly
    drivetrain.set_speed(5, 5)

    # Drive forward until both sensors detect the line
    while not is_over_line(line_threshold):
        pass # This line does nothing. The loop continues as long as
             # the is_over_line function returns False (not over the line)

    # Stop the drivetrain
    drivetrain.stop()

In this modified code, the `while` loop now uses our `is_over_line()` function to check if the robot should stop. As long as `is_over_line(line_threshold)` returns `False` (meaning the robot is not over the line), the loop continues, and the robot keeps driving forward. Once the function returns `True` (both sensors are over the line), the loop stops, and the robot stops.

Test your code by placing the robot on a surface with a line and observing if it stops when both sensors are over the line. If the robot does not stop correctly, try adjusting your `line_threshold` value or carefully check the logic inside your ``is_over_line()`` function. Your XRP should now perform the same action as in the previous lesson, but with the line-sensing logic neatly packaged inside a function!

.. figure:: images/stop_at_line.webp

Staying Inside a Circle
-----------------------

Great job! You've now created and tested your `is_over_line` function. Let's move on to the main challenge: keeping the robot inside a circular boundary.

To make the robot stay within the circle, we will use our `is_over_line` function to detect when the robot reaches the dark line marking the circle's edge. When the robot detects this line, we want it to perform a sequence of actions to get back inside:

1.  **Stop moving forward.**
2.  **Back up** slightly to move away from the boundary.
3.  **Turn** a random angle to change its direction. We'll make it turn somewhere between 135 and 225 degrees.
4.  **Continue moving forward** in its new direction.

.. note::

    To make the robot turn at a random angle, we can use a special tool in Python called a **library**. A library is a collection of pre-written code that you can use in your own programs. We'll use the `random` library, which has functions for generating random numbers.

    Specifically, we'll use the `random.randint()` function. This function allows you to pick a random whole number (an integer) between two values that you specify.

    The way you use it is like this: `random.randint(start_value, end_value)`.

    * `start_value`: This is the smallest possible random number you want.
    * `end_value`: This is the largest possible random number you want.

    For example:

    .. code-block:: python

        import random  # This line tells Python we want to use the 'random' library

        # Pick a random whole number (angle in degrees) between 135 and 225
        angle = random.randint(135, 225)

        print(f"Your random angle is: {angle} degrees")

Here's the Python code that will make the XRP robot stay inside the circle:

.. code-block:: python

    from XRPLib.defaults import *
    import random  # We need this to generate random numbers

    def is_over_line(threshold):
        # Check the left sensor
        if reflectance.get_left() > threshold:
            left_over_line = True
        else:
            left_over_line = False

        # Check the right sensor
        if reflectance.get_right() > threshold:
            right_over_line = True
        else:
            right_over_line = False

        # Return True if both sensors are over the line
        return left_over_line and right_over_line

    line_threshold = 0.5  # Use the threshold value you determined!

    while True:  # This creates a loop that will run forever
        # Move the robot forward at a slow speed
        drivetrain.set_speed(5, 5)

        # Keep driving forward as long as the sensors are NOT over the line
        while not is_over_line(line_threshold):
            pass # Do nothing inside this loop, just keep driving

        # Once the loop above finishes, it means the robot has detected the line.
        # First, stop the robot
        drivetrain.stop()

        # Then, make the robot back up a little bit
        drivetrain.set_speed(-5, -5) # Negative speed means backward
        sleep(0.5)  # Drive backward for half a second
        drivetrain.stop()

        # Now, let's turn the robot by a random angle
        random_angle = random.randint(135, 225)  # Pick a random angle between 135 and 225 degrees
        drivetrain.turn_degrees(random_angle)

        # After turning, the outer 'while True' loop will start again,
        # and the robot will continue driving forward in its new direction.

.. admonition:: Try it out

    Run this code on your XRP robot. Place the robot inside a circle that you've drawn on the floor using a dark marker or tape. Observe how the robot moves. Does it successfully stay inside the circle?

    If you notice that the robot sometimes crosses the boundary line, you can try adjusting the `line_threshold` value. A slightly higher or lower threshold might work better depending on the darkness of your line and the lighting conditions.

    By combining line sensing with backing up and turning at random angles, you've programmed the robot to continuously detect the boundary and react in a way that keeps it moving inside the circle!

.. error::

    add a vid

**Recap**

Today, you have:

* Understood the benefits of using functions to organize and reuse code in Python.
* Learned how to define a function called ``is_over_line()`` that takes a threshold as a parameter and returns ``True`` if both reflectance sensors detect a line, and ``False`` otherwise.
* Practiced calling the ``is_over_line()`` function in your main program to check the robot's sensor state.
* Learned how to import and use the `random` library to generate random integer values for the robot's turning angle.
* Programmed the XRP robot to stay within a circular boundary by implementing the following behavior: drive forward, detect the line using the ``is_over_line()`` function, back up, turn by a random angle, and repeat.