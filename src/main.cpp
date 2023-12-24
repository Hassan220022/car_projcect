#include <Arduino.h>

// Assume that a higher sensor value means the sensor is over the line
const int threshold = 40;

// Ziegler-Nichols tuning rules
// float Ku = 250.0; // Ultimate gain
// float Tu = 300.0; // Ultimate period

// Define the motor control pins
const int sensor_power = 11;
const int left_motor_forward = 9;
const int left_motor_backward = 8;
const int right_motor_backward = 7;
const int right_motor_forward = 6;

// defalut motor speed
const int motor_speed = 250;
int variable_speed = 0.0;

// Define enable pins
const int enable_left = 10;
const int enable_right = 5;

// Define the IR sensor pins
const int orang_sensor = A1;
const int green_sensor = A3;

// constant for PID control
//  Ziegler-Nichols tuning rules for PID controller
// const float Kp = 0.6 * Ku;	  // Proportional constant
// const float Ki = 2 * Kp / Tu; // Integral constant
// const float Kd = Kp * Tu / 8; // Derivative constant

// PID constants
const float kp = 100;
const float ki = 1;
const float kd = 150;

// PID variables
float Previous_error = 0.0;
float Integral = 0.0;
float Integral_limit = 100.0;
int Reset_threshold = 50;
int count = 50;

// Define the modes
const int MODE_RC = 0;
const int MODE_AUTO = 1;

// Start in RC mode
int mode = MODE_RC;

float posistion(int left_value_orange, int right_value_green)
{
	if (left_value_orange == LOW && right_value_green == LOW)
	{
		return 0.30;
	}
	else if (left_value_orange == LOW)
	{
		return 0.15;
	}
	else if (right_value_green == LOW)
	{
		return 0.45;
	}
}
void error_calc(int var1, int var2, float error, int counter)
{
	if (counter != 0)
	{
		counter = 50;
		// on the track - move forword
		moveForward(var1, var2);
	}
	else if (error < 0)
	{
		// turn left
		counter = 50;
		turnLeft(var1, var2);
	}
	else if (error > 0)
	{
		// turn right
		counter = 50;
		turnRight(var1, var2);
	}
	else
	{
		if (counter > 0)
		{
			counter--;
			moveForward(var1, var2);
		}
		else
		{
			// lost track - stop
			stop();
		}
	}
}

void PID(int orange_value, int green_value)
{
	// Calculate error (difference from the desired position)
	float error = posistion(orange_value, green_value) - 0.30;

	// PID control to adjust motor speeds based on error
	float proportional = kp * error;
	Integral += ki * error;

	float derivative = kd * (error - Previous_error);

	int pid = int(proportional + Integral + derivative);

	// Integral wind-up protection
	if (abs(error) > Reset_threshold)
	{
		Integral = 0.0;
	}

	// update motor speed
	int var1 = motor_speed + pid;
	int var2 = motor_speed - pid;

	// Ensure motor speeds are within valid range
	var1 = constrain(var1, 100, 255);
	var2 = constrain(var2, 100, 255);

	// Update previous error for the next iteration
	Previous_error = error;

	// Line following logic
	error_calc(var2, var1, error, count);
}

void moveForward()
{
	digitalWrite(left_motor_forward, HIGH);
	digitalWrite(left_motor_backward, LOW);
	digitalWrite(right_motor_forward, HIGH);
	digitalWrite(right_motor_backward, LOW);

	analogWrite(enable_left, variable_speed);  // Adjust the speed as needed
	analogWrite(enable_right, variable_speed); // Adjust the speed as needed
}
void moveForward(int var1, int var2)
{
	digitalWrite(left_motor_forward, HIGH);
	digitalWrite(left_motor_backward, LOW);
	digitalWrite(right_motor_forward, HIGH);
	digitalWrite(right_motor_backward, LOW);

	analogWrite(enable_left, var1);	 // Adjust the speed as needed
	analogWrite(enable_right, var2); // Adjust the speed as needed
}

void moveBackward()
{
	digitalWrite(left_motor_forward, LOW);
	digitalWrite(left_motor_backward, HIGH);
	digitalWrite(right_motor_forward, LOW);
	digitalWrite(right_motor_backward, HIGH);

	analogWrite(enable_left, variable_speed);  // Adjust the speed as needed
	analogWrite(enable_right, variable_speed); // Adjust the speed as needed
}

void moveBackward(int var1, int var2)
{
	digitalWrite(left_motor_forward, LOW);
	digitalWrite(left_motor_backward, HIGH);
	digitalWrite(right_motor_forward, LOW);
	digitalWrite(right_motor_backward, HIGH);

	analogWrite(enable_left, var1);	 // Adjust the speed as needed
	analogWrite(enable_right, var2); // Adjust the speed as needed
}

void turnRight()
{
	digitalWrite(left_motor_forward, LOW);
	digitalWrite(left_motor_backward, HIGH);
	digitalWrite(right_motor_forward, HIGH);
	digitalWrite(right_motor_backward, LOW);

	analogWrite(enable_left, variable_speed);  // Adjust the speed as needed
	analogWrite(enable_right, variable_speed); // Adjust the speed as needed
}

void turnRight(int var1, int var2)
{
	digitalWrite(left_motor_forward, LOW);
	digitalWrite(left_motor_backward, HIGH);
	digitalWrite(right_motor_forward, HIGH);
	digitalWrite(right_motor_backward, LOW);

	analogWrite(enable_left, var1);	 // Adjust the speed as needed
	analogWrite(enable_right, var2); // Adjust the speed as needed
}

void turnLeft()
{
	digitalWrite(left_motor_forward, HIGH);
	digitalWrite(left_motor_backward, LOW);
	digitalWrite(right_motor_forward, LOW);
	digitalWrite(right_motor_backward, HIGH);

	analogWrite(enable_left, variable_speed);  // Adjust the speed as needed
	analogWrite(enable_right, variable_speed); // Adjust the speed as needed
}

void turnLeft(int var1, int var2)
{
	digitalWrite(left_motor_forward, HIGH);
	digitalWrite(left_motor_backward, LOW);
	digitalWrite(right_motor_forward, LOW);
	digitalWrite(right_motor_backward, HIGH);

	analogWrite(enable_left, var1);	 // Adjust the speed as needed
	analogWrite(enable_right, var2); // Adjust the speed as needed
}

void stop()
{
	digitalWrite(left_motor_forward, LOW);
	digitalWrite(left_motor_backward, LOW);
	digitalWrite(right_motor_forward, LOW);
	digitalWrite(right_motor_backward, LOW);
	analogWrite(enable_left, 0);
	analogWrite(enable_right, 0);
}

void setup()
{
	pinMode(sensor_power, OUTPUT);
	// Set up the IR sensors
	pinMode(orang_sensor, INPUT);

	pinMode(green_sensor, INPUT);

	// Set up the motors
	pinMode(left_motor_forward, OUTPUT);
	pinMode(left_motor_backward, OUTPUT);
	pinMode(right_motor_forward, OUTPUT);
	pinMode(right_motor_backward, OUTPUT);

	// Set up the Bluetooth module
	Serial.begin(9600);

	// Set initial motor speed
	variable_speed = motor_speed - variable_speed;
	analogWrite(enable_left, variable_speed);  // Adjust the speed as needed
	analogWrite(enable_right, variable_speed); // Adjust the speed as needed
}

void loop()
{
	// Check for Bluetooth commands
	if (Serial.available() > 0)
	{
	start:
		char command = Serial.read();
		Serial.print(command);

		if (mode == MODE_RC)
		{
			if (command == 'f' || command == 'F')
			{
				moveForward();
			}
			else if (command == 'B' || command == 'b')
			{
				moveBackward();
			}
			else if (command == 'L' || command == 'l')
			{
				turnLeft();
			}
			else if (command == 'R' || command == 'r')
			{
				turnRight();
			}
			else if (command == 'D' || command == 'd' || command == '0')
			{
				stop();
			}
			else if (command == '1')
			{
				variable_speed = 10 * 2;
			}
			else if (command == '2')
			{
				variable_speed = 20 * 2;
			}
			else if (command == '3')
			{
				variable_speed = 30 * 2;
			}
			else if (command == '4')
			{
				variable_speed = 40 * 2;
			}
			else if (command == '5')
			{
				variable_speed = 50 * 2;
			}
			else if (command == '6')
			{
				variable_speed = 60 * 2;
			}
			else if (command == '7')
			{
				variable_speed = 70 * 2;
			}
			else if (command == '8')
			{
				variable_speed = 80 * 2;
			}
			else if (command == '9')
			{
				variable_speed = 90 * 2;
			}
			else if (command == 'q' || command == 'Q')
			{
				variable_speed = 100 * 2;
			}
			else if (command == '0')
			{
				stop();
			}
			else if (command == 'w' || command == 'W')
			{
				// Switch mode
				mode = (mode == MODE_RC) ? MODE_AUTO : MODE_RC;
			}
		}
		else if (mode == MODE_AUTO)
		{
			digitalWrite(sensor_power, HIGH);
			variable_speed = 0;
			// Read sensor values
			int left_value_orange = digitalRead(orang_sensor);
			int right_value_green = digitalRead(green_sensor);

			// PID function
			PID(left_value_orange, right_value_green);
		}
	}
}
