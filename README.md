# Line Following Robot

This project is about a line following robot that uses an Arduino microcontroller. The robot uses infrared sensors to detect a line and follows it using a PID controller for precise movement.

## Components

The main components of the robot include:

- Arduino microcontroller
- Infrared sensors
- DC motors
- Motor driver

## How It Works

The robot uses two infrared sensors to detect a line. The sensors are positioned such that one is on each side of the line. The robot moves forward when both sensors detect the line. If the left sensor detects the line and the right sensor does not, the robot turns left. Conversely, if the right sensor detects the line and the left sensor does not, the robot turns right.

The speed of the motors is controlled using a PID controller. The PID controller takes the difference between the desired position (on the line) and the current position as input, and calculates the necessary adjustments to the motor speeds.

The robot also has a remote control mode, where it can be controlled using Bluetooth commands.

## Code Structure

The code is structured into several functions:

- `PID()`: This function calculates the error between the desired position and the current position, and adjusts the motor speeds using a PID controller.
- `moveForward()`, `moveBackward()`, `turnRight()`, `turnLeft()`: These functions control the movement of the robot.
- `stop()`: This function stops the robot.
- `setup()`: This function sets up the Arduino pins and initializes the serial communication for the Bluetooth module.
- `loop()`: This function is the main loop of the Arduino program. It checks for Bluetooth commands and controls the robot accordingly.

## Usage

Upload the code to your Arduino board using the Arduino IDE. Make sure to adjust the pin numbers and PID constants to match your hardware setup.

## Note

The PID constants in the code are set using the Ziegler-Nichols tuning method. This method provides a good starting point, but the constants may need to be fine-tuned for your specific robot and environment.
