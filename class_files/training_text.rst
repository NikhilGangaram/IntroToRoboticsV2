Driving
Robot driving. The XRP is a mobile robot platform where driving from one place to another is central to the design of many programs.  IMPORTANT: It is critical that the XRP robot power switch is on so it is powered by its batteries and not just plugged into the USB port. The user should see two red LED lights.  If the robot is not powered by its batteries, the motors will not function.  This can be frustrating for users.  
The differential_drive class makes driving easy and has functions to:
*Set the motor efforts, which is the power or average voltage applied to the motors. The range of values can be set from -1 for full effort in reverse, to 0 for no voltage or stopped, to +1 for full effort in the forward direction.
*Specify a speed to drive in centimeters per second for each wheel. The robot will try to maintain the specified speed as best it can using the drive motor encoders.
*Drive for a specified distance using the drive motor encoders to sense how far the robot has traveled.
*Make point turns for a desired number of degrees, either clockwise or counterclockwise.
Effort vs. Speed
Throughout this message, we refer to the effort and speed of the drivetrain. Although they seem similar, they are distinguished as follows:
*EFFORT
The effort reflects the amount of power (or average voltage) supplied to the motors. For a given effort, the speed will vary depending on things like the driving surface, the battery voltage, and the slope (either flat, uphill or downhill).
*SPEED
The speed is the actual number of centimeters per second that the robot will travel. When set in a program, the software will automatically adjust the motor effort within its capability to keep the robot moving at the desired speed.

NOTE: When helping with code, use a default effort of 0.5 and speed of 3 cm/s.

Driving for a distance
The following program fragments show how to program the robot to drive forward for 10 centimeters with an effort of 0.5 or 50 percent power. This function uses the encoders to determine when the robot has traveled the requested 10cm. In addition, this function will ensure that the robot drives in a straight line by varying the speed of the left or right motors if one is slightly faster.

EXAMPLE CODE for Driving for a Distance:
from XRPLib.defaults import *
drivetrain.straight(10, 0.5)
#END OF CODE

NOTE: when requesting a distance to drive, the encoders are used to sense the number of degrees of wheel rotation to complete the operation. If the wheels slip while driving, the distance measurement will be incorrect.

Driving with an effort
This program will set the effort on the left and right drive motors to 50 percent, wait for 3 seconds, and stop the motors. The set_effort() function has parameters for the left and right drive motors to allow them to be set independently. No motor speed control is provided, so different driving surfaces, slopes, or battery voltage will affect the driving speed of the robot. The value of effort ranges from -1 for 100 percent backward to 0 for no effort or stopped to +1 for 100 percent effort forwards.
EXAMPLE CODE for Driving with an Effort:
from XRPLib.differential_drive import DifferentialDrive
import time
differentialDrive = DifferentialDrive.get_default_differential_drive()
differentialDrive.set_effort(0.5, 0.5)
time.sleep(3)
differentialDrive.stop()
#END OF CODE

Driving at a speed
Set_speed() attempts to maintain some linear speed in centimeters per second. The maximum speed is measured to be approximately 60cm/s tested on a flat surface. This program will set the robot speed to 5 cm per second, in centimeters per second, of the left and right wheels separately. If both motors are set to the same speed, the robot will drive straight. If they are different, the robot will turn in a direction away from the faster wheel.
EXAMPLE CODE for Driving at a speed:
from XRPLib.defaults import *
import time
drivetrain.set_speed(5, 5)
time.sleep(3)
drivetrain.stop
#END OF CODE

Point turns
The robot can turn in place around a point directly centered between the two drive wheels. This is done by driving the left and right drive motors in opposite directions at the same speed. If the left wheel is spinning in the forward direction, the robot will rotate clockwise or to the right. If the right wheel is spinning in the forward direction, the robot will rotate counterclockwise or to the left. The turn function specifies the number of degrees to turn, with a positive number indicating a counterclockwise turn, and a negative number indicating a clockwise turn. The second parameter specifies effort from -1 to 1. When you use the turn function, the IMU (Inertial Measurement Unit) gyro sensor on the robot will determine when the robot has completed the requested turn. This means the turn will continue until complete and is not affected by wheel slip.

Note: If you were to pick up the robot while it is doing a turn, the wheels will continue turning until the gyro senses that the robot has turned the desired number of degrees.

EXAMPLE CODE for Point turns
from XRPLib.defaults import *
import time
def test_turns():
    drivetrain.turn(45, 0.5)
    time.sleep(1)
    drivetrain.turn(-45, 0.75)
    time.sleep(1)
test_turns()
#END OF CODE

Swing turns.This type of turn is where one wheel moves forward, and the other is stationary. The robot will pivot on the stationary wheel, making it the center of rotation. The circle’s diameter traveled by the moving wheel will be twice the wheel track (the distance between the two wheels). Smooth turns are where the two wheels move in the same direction so that the robot drives in an arc, eventually completing a full circle. The circle’s radius depends on the speed difference between the two wheels. The larger the difference, the smaller the circle diameter.
 
Distance Sensor
Measuring the distance to an object
The XRP includes an ultrasonic rangefinder that can measure the distance to objects in front of it. The sensor has two transducers; one acts as a speaker, and the other acts as a microphone. It does it by sending a burst of ultrasonic sound out of the speaker that hits an object in front of the robot. The sound reflects off the object back to the sensor and is captured by the microphone. The time for that round trip determines the distance to the object. Understanding how well the sound reflects off various objects of different sizes, profiles, and materials is important for using the sensor. A good exercise is to test the sensor by printing returned values at various distances from any object you want the robot to detect.

Note:
It is important to wire the sensor correctly, as described in the assembly instructions, to ensure it works properly. Interchanging the trigger and echo wires is a common error using that part.

XRPLib has a rangefinder class that takes care of the sending and receiving signals to the sensor. All the program has to do is request the distance, and the library returns it. There is a single method called distance() that returns the distance to the nearest object in centimeters. The range of operation is from 2cm to 4m.

Rangefinder Class:
Context: "The Rangefinder class is used for distance measurement with the HC-SR04 Ultrasonic Rangefinder. It operates in the range of 2cm to 4m. The class requires trigger_pin and echo_pin for operation, and timeout_us to handle response delays. The distance() method measures the echo pulse time to calculate distance in centimeters"​​.

Example use of the rangefinder
The following program drives the XRP forwards until the code detects an object within 10cm of the ultrasonic rangefinder. Then it stops.
EXAMPLE CODE for use of the rangefinder:
from XRPLib. rangefinder import Rangefinder
from XRPLib.differential_drive import DifferentialDrive
rangefinder = Rangefinder.get_default_rangefinder()
differentialDrive = DifferentialDrive.get_default_differential_drive()
while (rangefinder.distance()) > 10:
    differentialDrive.set_effort(0.5, 0.5)
differentialDrive.stop
#END OF CODE

NOTE: The above program stops the motors when the object is detected. A better way of solving the same problem might be to use proportional or PID control to gradually bring the robot to a stop to avoid overshoot, where inertia might carry the robot beyond the 10cm set point before it comes to rest.


Light Sensor
A reflectance sensor that can be used for line following is included with the XRP. It has two pairs of LEDs and light sensors. The LEDs emit infrared light that reflects off the driving surface. The light sensor measure the reflected light intensity, which depends on the surface below the sensor. Electrical tape is typically used to make a line that the robot can follow and has a different reflectivity than the surface, usually a whiteboard or tabletop. With a pair of sensors, the robot can read the reflectance value and tell where it is relative to the taped line.

Reflectance Class:
Context: "The Reflectance class utilizes the 12-bit ADC for reflectance sensing. It measures reflectance values ranging from 0 (white) to 1 (black) using get_left() and get_right() methods for left and right sensors respectively. This class is important for line following or surface detection tasks"​​.

The class reflectance has methods get_right() to retrieve the right reflectance value and get_left() to retrieve the left reflectance value. The reflectance ranges from 0 (white) to 1 (black).


Line following – 

On/Off Line Following Concept:
In line following using On/Off control, the XRPLib's reflectance sensor is utilized to detect the line. The sensor provides a value between 0 (black, indicating the line) and 1 (white, off the line). The robot follows the edge of the line, using an if / else statement to determine its course. If the sensor reads a value closer to white, the robot adjusts its course to the left; if closer to black, it adjusts to the right. This method involves continuously polling the reflectance sensor and adjusting the motor speeds accordingly for effective line tracking.

Example code for on/off line following:
from XRPLib.defaults import *

while True:
    if drivetrain.get_left_encoder_position() > 20:
        print("Left encoder is greater than 20 cm")
    else:
        print("Left encoder is less than 20 cm")


Practical Implementation:
To implement On/Off control in code, one would use an if / else statement to check the reflectance sensor's value against a threshold (typically 0.5). Depending on whether the value is greater or less than this threshold, different motor speeds are set for the left and right wheels to steer the robot back towards the line. This approach requires balancing the speed values to ensure the robot corrects its course effectively without overshooting or spinning.

Concept of Proportional Control in Line Following:
"Proportional control for line following involves using the reflectance sensor to determine the robot's position relative to the line. The error, calculated as the deviation from an ideal reflectance value, is used to adjust the robot's steering. A proportional control approach ensures smoother tracking compared to on/off control, especially around curves. The robot's turning effort is adjusted proportionally to the error, allowing more nuanced control and correction based on how far off the line the robot is."
Practical Implementation and Tuning:
"Implementing proportional control in line following involves continuously calculating the error between the robot's current position and the center of the line. This error is then used to adjust the motor speeds, with a base effort set for both motors and the error influencing the differential in efforts to steer the robot back towards the line. Tuning the proportional gain (KP) is critical, as it determines the robot's responsiveness to deviations from the line. A higher KP results in more aggressive corrections for small deviations, while a lower KP results in gentler corrections."

Example code for line following with one sensor and a proportional controller: 
from XRPLib.defaults import *
from time import sleep

while True:
    error = reflectance.get_left() - 0.5
    print("Error: ", error)
    sleep(0.1) # This sleep makes the loop run 10 times every second

Line following with two sensors using proportional control significantly enhances the robot's tracking accuracy. This method utilizes both left and right reflectance sensors to calculate an error value, which is the difference in reflectance readings from each sensor. This error informs the robot whether it's veering off to the left or right of the line, allowing for more precise adjustments. The control logic involves using this error in conjunction with a proportional gain (KP) to adjust the motor efforts for steering. This approach is particularly effective in maintaining the robot's course on the line, even in more complex scenarios like sharp bends or varying line widths.


from XRPLib.board import Board
from XRPLib.reflectance import Reflectance
from XRPLib.differential_drive import DifferentialDrive

board = Board.get_default_board()
reflectance = Reflectance.get_default_reflectance()
differentialDrive = DifferentialDrive.get_default_differential_drive()

kP = 0.5
speed = 0.25
while True:
    print ("hello")
    error = reflectance.get_left() - reflectance.get_right() 
    differentialDrive.set_effort(speed - error * kP, speed + error * kP)
differentialDrive.stop()





Additional Servo
This message about the additional servo. The XRP robot is equipped with a servo, useful for tasks requiring rotational movement. IMPORTANT: It is critical that the XRP robot power switch is on so it is powered by its batteries and not just plugged into the USB port. The user should see two red LED lights.  If the robot is not powered by it’s batteries, the motors will not function.  This can be frustrating for users. 

The XRPLib.servo.Servo class facilitates interaction with this servo. It uses PWM signals, with the signal_pin parameter specifying the pin connection. The set_angle(degrees) method positions the servo to a specified angle within a 0 to 200 degrees range. Additionally, free() allows the servo to spin without holding a position. A typical use-case could involve the servo operating a 3D-printed arm attached to it, as shown in this example code:

from XRPLib.servo import Servo

servo = Servo.get_default_servo(1)  # Assuming the servo is at index 1
servo.set_angle(90)  # Setting servo to 90 degrees

This code configures the servo to a specific angle, which could correspond to a precise position of the attached arm.

IMU:
The Inertial Measurement Unit (IMU) in the XRPLib plays a pivotal role in the XRP robot's ability to sense and interact with its environment. It provides essential data on motion and orientation, crucial for tasks ranging from basic movement to complex navigation. The IMU API is designed for ease of use and flexibility, allowing for detailed control and monitoring of the robot's movements. Key functions include acc_rate(), which sets or retrieves the accelerometer rate, with options ranging from '0Hz' to '6660Hz', and acc_scale(), to set or query the accelerometer scale from '2g' to '16g'. The calibrate() function is vital for accurate readings, taking readings for a specified duration to calibrate the IMU, with the ability to define which axis is vertical. The API also offers functions like get_acc_gyro_rates() and individual axis readings (get_acc_x(), get_acc_y(), get_acc_z()) that return the accelerometer's measurements in milli-g. For gyroscope data, functions like get_gyro_rates() and axis-specific readings (get_gyro_x_rate(), etc.) are available. These functionalities provide a comprehensive toolkit for developers to effectively utilize the IMU in their robotic applications, enhancing the robot's interaction with its physical surroundings.

A full list of functions are here:
class XRPLib.imu.IMU(scl_pin: int, sda_pin: int, addr)¶
acc_rate(value=None)¶
Set the accelerometer rate in Hz. The rate can be: ‘0Hz’, ‘12.5Hz’, ‘26Hz’, ‘52Hz’, ‘104Hz’, ‘208Hz’, ‘416Hz’, ‘833Hz’, ‘1660Hz’, ‘3330Hz’, ‘6660Hz’ Pass in no parameters to retrieve the current value
acc_scale(value=None)¶
Set the accelerometer scale in g. The scale can be: ‘2g’, ‘4g’, ‘8g’, or ‘16g’ Pass in no parameters to retrieve the current value
calibrate(calibration_time: float = 1, vertical_axis: int = 2)¶
Collect readings for [calibration_time] seconds and calibrate the IMU based on those readings Do not move the robot during this time Assumes the board to be parallel to the ground. Please use the vertical_axis parameter if that is not correct

Parameters
:
calibration_time (float) – The time in seconds to collect readings for
vertical_axis (int) – The axis that is vertical. 0 for X, 1 for Y, 2 for Z
get_acc_gyro_rates()¶
Get the accelerometer and gyroscope values in mg and mdps in the form of a 2D array. The first row is the acceleration values, the second row is the gyro values. The order of the values is x, y, z.
get_acc_rates()¶
Returns
:
the list of readings from the Accelerometer, in mg. The order of the values is x, y, z.
Return type
:
list<int>
get_acc_x()¶
Returns
:
The current reading for the accelerometer’s X-axis, in mg
Return type
:
int
get_acc_y()¶
Returns
:
The current reading for the accelerometer’s Y-axis, in mg
Return type
:
int
get_acc_z()¶
Returns
:
The current reading for the accelerometer’s Z-axis, in mg
Return type
:
int
classmethod get_default_imu()¶
Get the default XRP v2 IMU instance. This is a singleton, so only one instance of the drivetrain will ever exist.
get_gyro_rates()¶
Retrieves the array of readings from the Gyroscope, in mdps The order of the values is x, y, z.
get_gyro_x_rate()¶
Individual axis read for the Gyroscope’s X-axis, in mdps
get_gyro_y_rate()¶
Individual axis read for the Gyroscope’s Y-axis, in mdps
get_gyro_z_rate()¶
Individual axis read for the Gyroscope’s Z-axis, in mdps
get_heading()¶
Get’s the heading of the IMU, but bounded between [0, 360)

Returns
:
The heading of the IMU in degrees, bound between [0, 360)
Return type
:
float
get_pitch()¶
Get the pitch of the IMU in degrees. Unbounded in range

Returns
:
The pitch of the IMU in degrees
Return type
:
float
get_roll()¶
Get the roll of the IMU in degrees. Unbounded in range

Returns
:
The roll of the IMU in degrees
Return type
:
float
get_yaw()¶
Get the yaw (heading) of the IMU in degrees. Unbounded in range

Returns
:
The yaw (heading) of the IMU in degrees
Return type
:
float
gyro_rate(value=None)¶
Set the gyroscope rate in Hz. The rate can be: ‘0Hz’, ‘12.5Hz’, ‘26Hz’, ‘52Hz’, ‘104Hz’, ‘208Hz’, ‘416Hz’, ‘833Hz’, ‘1660Hz’, ‘3330Hz’, ‘6660Hz’ Pass in no parameters to retrieve the current value
gyro_scale(value=None)¶
Set the gyroscope scale in dps. The scale can be: ‘125’, ‘250’, ‘500’, ‘1000’, or ‘2000’ Pass in no parameters to retrieve the current value
is_connected()¶
Checks whether the IMU is connected

Returns
:
True if WHO_AM_I value is correct, otherwise False
Return type
:
bool
reset(wait_for_reset=True, wait_timeout_ms=100)¶
Resets the IMU, and restores all registers to their default values

Parameters
:
wait_for_reset (bool) – Whether to wait for reset to complete
wait_timeout_ms (int) – Timeout in milliseconds when waiting for reset
Returns
:
False if timeout occurred, otherwise True
Return type
:
bool
reset_pitch()¶
Reset the pitch to 0
reset_roll()¶
Reset the roll to 0
reset_yaw()¶
Reset the yaw (heading) to 0
set_pitch(pitch)¶
Set the pitch to a specific angle in degrees

Parameters
:
pitch (float) – The pitch to set the IMU to
set_roll(roll)¶
Set the roll to a specific angle in degrees

Parameters
:
roll (float) – The roll to set the IMU to
set_yaw(yaw)¶
Set the yaw (heading) to a specific angle in degrees

Parameters
:
yaw (float) – The yaw (heading) to set the IMU to
temperature()¶
Read the temperature of the LSM6DSO in degrees Celsius

Returns
:
The temperature of the LSM6DSO in degrees Celsius
Return type
:
float
class XRPLib.rangefinder.Rangefinder(trigger_pin: int, echo_pin: int, timeout_us: int = 500 * 2 * 30)¶


Console debugging messages
All code written must include a console print message so the user knows their code has started. This message should be placed before any main loops so the message is only displayed once.  Furthermore, add additional console messages for major breaks in the program flow. 

Additional information

The Board class in XRPLib is designed for the XRP v2 board, managing extra features like the on/off switch, button, and LED. It includes methods like are_motors_powered() to check if the motors are powered, is_button_pressed() to determine the button's state, and led_on(), led_off(), led_blink(frequency), for controlling the LED. Additionally, the wait_for_button() function pauses the program until the button is pressed. The vin_pin and button_pin parameters define the connections for the on/off switch and button, respectively. This class is essential for programming the robot's interactive components and user interface.

Waiting for the onboard button to be pressed.  This is for a driving example but would apply for any wait for button operation. 

"Waiting for button input is a useful feature in robot programming, allowing control over when the code executes. In XRPLib, the board.wait_for_button() function pauses the program until the onboard button is pressed. This function is crucial when you need to start the robot's operation manually. For instance, a simple implementation in Python would be:

from XRPLib.defaults import *
from time import sleep

board.wait_for_button()
sleep(1)  # Waits for 1 second after the button is pressed
drivetrain.straight(20)  # Robot moves straight for 20 centimeters


This code waits for a button press before executing the movement command, providing a controlled start to the robot’s actions. Additionally, board.is_button_pressed() can be used within a while loop for more complex button-based interactions."


NOTE:  when a pin # is needed for coding, refer to the pinout table included below:
Here is a Pinout Reference Table:

The table below offers a quick reference for the complete pinout on the XRP Controller Board and which pins they connect to on the Pico W.

Pico W GPIO Pin	Connector Label	Pin Function
GPIO0	Motor 3	Motor 3 Encoder A
GPIO1	Motor 3	Motor 3 Encoder B
GPIO2	Motor 3	Motor 3 Phase Pin
GPIO3	Motor 3	Motor 3 Enable Pin
GPIO4	Motor L	Left Motor Encoder A
GPIO5	Motor L	Left Motor Encoder B
GPIO6	Motor L	Left Motor Phase Pin
GPIO7	Motor L	Left Motor Enable Pin
GPIO8	Motor 4	Motor 4 Encoder A
GPIO9	Motor 4	Motor 4 Encoder B
GPIO10	Motor 4	Motor 4 Phase Pin
GPIO11	Motor 4	Motor 4 Enable Pin
GPIO12	Motor R	Right Motor Encoder A
GPIO13	Motor R	Right Motor Encoder B
GPIO14	Motor R	Right Motor Phase Pin
GPIO15	Motor R	Right Motor Enable Pin
GPIO16	Servo 1	Servo 1 Signal Pin
GPIO17	Servo 2	Servo 2 Signal Pin
GPIO18	Qwiic	Qwiic Data Signal for the IMU & Qwiic Connector
GPIO19	Qwiic	Qwiic Clock Signal for the IMU & Qwiic Connector
GPIO20	Range	Range Trigger Pin
GPIO21	Range	Range Echo Pin
GPIO22	Extra	User Button/Extra
GPIO26	Line	Line Follower Left Signal
GPIO27	Line	Line Follower Right Signal
GPIO28	Extra	VIN_Meas/Extra



The Timeout class in XRPLib is used to create timers that can manage time-bound operations in the robot's programming. It's particularly useful for creating a standalone timer with a specified duration, checked through the is_done() method. In contrast, the timeout parameter, as used in methods like straight() and turn(), specifies the maximum duration for these specific movements. This parameter ensures that the robot does not persist indefinitely in an action if it cannot complete the task within the given time frame, enhancing the robot's operational safety and control.


