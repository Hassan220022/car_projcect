#include <Arduino.h>

// Assume that a higher sensor value means the sensor is over the line
const int threshold = 40; 

// Ziegler-Nichols tuning rules
float Ku = 0.0; // Ultimate gain
float Tu = 0.0; // Ultimate period

// Define the motor control pins
const int sensor_power = 11;
const int left_motor_forward =   9;
const int left_motor_backward =  8;
const int right_motor_backward = 7;
const int right_motor_forward=   6;

//defalut motor speed
const int motor_speed = 250;
int variable_speed = 0.0;

//Define enable pins
const int enable_left = 10;
const int enable_right = 5;

// Define the IR sensor pins
const int orang_sensor = A1;
const int green_sensor = A3;

//constant for PID control
// Ziegler-Nichols tuning rules for PID controller
const float Kp = 0.6 * Ku; // Proportional constant
const float Ki = 2 * Kp / Tu;// Integral constant
const float Kd = Kp * Tu / 8;// Derivative constant

// PID variables
double Previous_error = 0.0;
double Integral = 0.0;
float Integral_limit = 100.0;
int Reset_threshold = 50;


// Define the modes
const int MODE_RC = 0;
const int MODE_AUTO = 1;

// Start in RC mode
int mode = MODE_RC;


void PID(int orange_value, int green_value)
{
	// Calculate error (difference from the desired position)

	int error = (orange_value + green_value) / 2 - threshold;

	// PID control to adjust motor speeds based on error

	float proportional = Kp * error;
	Integral += Ki * error;

	// Integral wind-up protection

	if (abs(error) > Reset_threshold)
	{
		Integral = 0.0;
	}

	// Limit the integral (optional)

	Integral = constrain(Integral, -Integral_limit, Integral_limit);
	float derivative = Kd * (error - Previous_error);
	variable_speed = motor_speed + proportional + Integral + derivative;

	// Ensure motor speeds are within valid range

	variable_speed = constrain(variable_speed, 0, 255);

	// Update previous error for the next iteration

	Previous_error = error;
}

void moveForward()
{
	digitalWrite(left_motor_forward, HIGH);
	digitalWrite(left_motor_backward, LOW);
	digitalWrite(right_motor_forward, HIGH);
	digitalWrite(right_motor_backward, LOW);

	analogWrite(enable_left, variable_speed);	// Adjust the speed as needed
	analogWrite(enable_right, variable_speed);	// Adjust the speed as needed	
}
void moveForward(int variable_speed)
{
	digitalWrite(left_motor_forward, HIGH);
	digitalWrite(left_motor_backward, LOW);
	digitalWrite(right_motor_forward, HIGH);
	digitalWrite(right_motor_backward, LOW);

	analogWrite(enable_left, variable_speed);	// Adjust the speed as needed
	analogWrite(enable_right, variable_speed);	// Adjust the speed as needed	
}


void moveBackward()
{
	digitalWrite(left_motor_forward, LOW);
	digitalWrite(left_motor_backward, HIGH);
	digitalWrite(right_motor_forward, LOW);
	digitalWrite(right_motor_backward, HIGH);

	analogWrite(enable_left, variable_speed);	// Adjust the speed as needed
	analogWrite(enable_right, variable_speed);	// Adjust the speed as needed	
}

void moveBackward(int variable_speed)
{
	digitalWrite(left_motor_forward, LOW);
	digitalWrite(left_motor_backward, HIGH);
	digitalWrite(right_motor_forward, LOW);
	digitalWrite(right_motor_backward, HIGH);

	analogWrite(enable_left, variable_speed);	// Adjust the speed as needed
	analogWrite(enable_right, variable_speed);	// Adjust the speed as needed	
}


void turnRight()
{
	digitalWrite(left_motor_forward, LOW);
	digitalWrite(left_motor_backward, HIGH);
	digitalWrite(right_motor_forward, HIGH);
	digitalWrite(right_motor_backward, LOW);

	analogWrite(enable_left, variable_speed);	// Adjust the speed as needed
	analogWrite(enable_right, variable_speed);	// Adjust the speed as needed	
}

void turnRight(int variable_speed)
{
	digitalWrite(left_motor_forward, LOW);
	digitalWrite(left_motor_backward, HIGH);
	digitalWrite(right_motor_forward, HIGH);
	digitalWrite(right_motor_backward, LOW);

	analogWrite(enable_left, variable_speed);	// Adjust the speed as needed
	analogWrite(enable_right, variable_speed);	// Adjust the speed as needed	
}

void turnLeft()
{
	digitalWrite(left_motor_forward, HIGH);
	digitalWrite(left_motor_backward, LOW);
	digitalWrite(right_motor_forward, LOW);
	digitalWrite(right_motor_backward, HIGH);

	analogWrite(enable_left, variable_speed);	// Adjust the speed as needed
	analogWrite(enable_right, variable_speed);	// Adjust the speed as needed	
}

void turnLeft(int variable_speed)
{
	digitalWrite(left_motor_forward, HIGH);
	digitalWrite(left_motor_backward, LOW);
	digitalWrite(right_motor_forward, LOW);
	digitalWrite(right_motor_backward, HIGH);

	analogWrite(enable_left, variable_speed);	// Adjust the speed as needed
	analogWrite(enable_right, variable_speed);	// Adjust the speed as needed	
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
	variable_speed = motor_speed-variable_speed;
	analogWrite(enable_left, variable_speed);	// Adjust the speed as needed
	analogWrite(enable_right, variable_speed);	// Adjust the speed as needed
}

void loop()
{
// Check for Bluetooth commands
	if (Serial.available() > 0)
	{
		start:
		char command = Serial.read();
		Serial.print(command);

		if(mode == MODE_RC)
		{
			if ( command=='f' || command == 'F')
			{
					moveForward();
			}
			else if ( command=='B' || command == 'b')
			{
				moveBackward();
			}
			else if ( command=='L' || command == 'l')
			{
				turnLeft();
			}
			else if ( command=='R' || command=='r')
			{
				turnRight();
			}
			else if (command == 'D' || command=='d' || command == '0')
			{
				stop();
			}
			else if (command =='1')
			{
				variable_speed = 10*2;
			}
			else if(command =='2')
			{
				variable_speed = 20*2;
			}
			else if(command == '3')
			{
				variable_speed = 30*2;
			}
			else if(command == '4')
			{
				variable_speed = 40*2;
			}
			else if(command == '5')
			{
				variable_speed = 50*2;
			}
			else if(command == '6')
			{
				variable_speed = 60*2;
			}
			else if(command == '7')
			{
				variable_speed = 70*2;
			}
			else if(command == '8')
			{
				variable_speed = 80*2;
			}
			else if(command == '9')
			{
				variable_speed = 90*2;
			}
			else if (command == 'q' || command=='Q')
			{
				variable_speed = 100*2;
			}
			else if (command == '0')
			{
				stop();
			}
			else if (command == 'w' || command=='W')
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
			int center_left_value_orange = digitalRead(orang_sensor);
			int center_right_value_green = digitalRead(green_sensor);

			//PID function

			PID(center_left_value_orange, center_right_value_green);

			// Read sensor values and calculate PID output here
			if((center_right_value_green == 0)&&(center_left_value_orange == 0))
			{
			moveForward(); 
			}
			//if Right Sensor is Black and Left Sensor is White then it will call turn Right function
			else if((center_right_value_green == 1)&&(center_left_value_orange == 0))
			{
				turnRight();
			}  
			//if Right Sensor is White and Left Sensor is Black then it will call turn Left function
			else if((center_right_value_green == 0)&&(center_left_value_orange == 1))
			{
				turnLeft();
			}
			else 
			{
				// Lost track - stop
				stop();
				goto start;
			}
		}
	}
}

