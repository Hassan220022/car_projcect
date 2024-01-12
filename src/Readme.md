# Arduino Line-Following Robot

This project is about a line-following robot controlled by an Arduino microcontroller. The robot uses infrared sensors to follow a line and can also be controlled via Bluetooth.

## Features

- Line-following mode: The robot follows a line using infrared sensors.
- Remote control mode: The robot can be controlled remotely via Bluetooth.

## Hardware Requirements

- Arduino microcontroller
- Infrared sensors
- DC motors
- Motor driver
- Bluetooth module (optional)

## Software Requirements

- Arduino IDE

## Setup

1. Connect the hardware as described in the hardware connections section.
2. Open the Arduino IDE and load the provided `main.cpp` file.
3. Compile and upload the program to your Arduino.

## Usage

Once the program is uploaded to the Arduino, the robot can be switched between line-following mode and remote control mode.

In line-following mode, the robot will automatically follow a line.

In remote control mode, the robot can be controlled via Bluetooth using the following commands:

- 'f' or 'F': Move forward
- 'b' or 'B': Move backward
- 'l' or 'L': Turn left
- 'r' or 'R': Turn right
- 'd' or 'D' or '0': Stop
- '1' to '9': Adjust speed
- 'q' or 'Q': Maximum speed
- 'w' or 'W': Switch mode

## Contributing

Contributions are welcome. Please open an issue to discuss your ideas before making changes.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.