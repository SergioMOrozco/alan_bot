#include <ros.h>
#include "Arduino.h"
#include <PID_v1.h>
#include <std_msgs/Float32.h>
#include <stdlib.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

// wheels
#define LEFT_PWM 6
#define LEFT_FR 7
#define RIGHT_PWM 5
#define RIGHT_FR 4

//encoders
#define RIGHT_ENCODER 2
#define LEFT_ENCODER 3

//prototypes 
void leftEncoder();
void rightEncoder();
void rightWheelCb( const std_msgs::Float32&);
void leftWheelCb( const std_msgs::Float32&);

//ROS variables
ros::NodeHandle nh;
ros::Subscriber<std_msgs::Float32> SubRight("wheel_power_right", &rightWheelCb);
ros::Subscriber<std_msgs::Float32> SubLeft("wheel_power_left", &leftWheelCb);

//PID variables
float Kp = 0.3, Ki =0.1 , Kd = 5000; //original

double LeftWheelState = 0, LeftWheelControlEffort = 0, LeftWheelSetPoint = 0;
double RightWheelState = 0, RightWheelControlEffort = 0, RightWheelSetPoint = 0;
PID RightWheelPidController (&RightWheelState, &RightWheelControlEffort, &RightWheelSetPoint, Kp, Ki, Kd, DIRECT); 
PID LeftWheelPidController (&LeftWheelState, &LeftWheelControlEffort, &LeftWheelSetPoint, Kp, Ki, Kd, DIRECT); 

//encoder variables
unsigned int LeftTick = 0;
unsigned int RightTick = 0;

//motors
int MinPwm = 115; //pwm where motor stops moving at 6v
int MaxPwm = 255;
int MaxRpm = 96; // max rpm at 6v with current load
int PulsesPerRevolution = 384;

bool LeftWheelBackward = false, RightWheelBackward = false;

void turnWheel(const std_msgs::Float32 &wheelPower, unsigned int pwmPin, unsigned int frPin) 
{
	//factor must be between -1 and 1
	float factor = max(min(wheelPower.data,1.0f),-1.0f);

	// Set our desired PWM that the PID Controller will work towards
	int pwm = map(abs(factor) ,0 ,1 ,MinPwm,MaxPwm);

	if ( pwmPin == LEFT_PWM)
	{
		LeftWheelBackward = factor < 0;
		LeftWheelSetPoint = pwm; 
	}
	else 
	{
		RightWheelBackward = factor < 0;
		RightWheelSetPoint = pwm; 
	}
}

void rightWheelCb( const std_msgs::Float32 &wheelPower)
{
	String message = "Wheel Power - Right: " + String(wheelPower.data);
	char *message_convert = message.c_str();
	nh.loginfo(message_convert);

	turnWheel(wheelPower,RIGHT_PWM,RIGHT_FR);
}
void leftWheelCb( const std_msgs::Float32 &wheelPower)
{
	String message = "Wheel Power - Left: " + String(wheelPower.data);
	char *message_convert = message.c_str();
	nh.loginfo(message_convert);

	turnWheel(wheelPower,LEFT_PWM,LEFT_FR);
}

void setup() 
{
	Serial.begin(115200);

	// Setup Timer
	// Reference: https://www.youtube.com/watch?v=IdL0_ZJ7V2s&t=110s
	TCCR1A = 0;
	TCCR1B = 0;
	OCR1A = 31249;
	OCR1B = 15624;
	TCCR1B |= (1<<CS12) | (1<<WGM12);
	TIMSK1 |= (1<<OCIE1A) | (1<<OCIE1B); //enable timer overflow

	// specifically for pin 5 and 6: Reference - https://www.etechnophiles.com/change-frequency-pwm-pins-arduino-uno/
	TCCR0B = TCCR0B & B11111000 | B00000010; // for PWM frequency of 7812.50 Hz

  	// initialize LED digital pin as an output.
  	pinMode(LED_BUILTIN, OUTPUT);
  	digitalWrite(LED_BUILTIN, HIGH);

	// hall effect sensor is an open drain output. This means that the
	// sensor will only pull line low, but it won't pull it high. It will just
	// leave it floating, so we need to pull it up to a voltage
	pinMode(LEFT_ENCODER,INPUT_PULLUP);
	pinMode(RIGHT_ENCODER,INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER),leftEncoder ,RISING);
	attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER),rightEncoder ,RISING);


	// wheels
  	pinMode(LEFT_PWM, OUTPUT);
  	pinMode(LEFT_FR, OUTPUT);
  	pinMode(RIGHT_PWM, OUTPUT);
  	pinMode(RIGHT_FR, OUTPUT);

  	// Init motors to stop
  	digitalWrite(LEFT_FR, LOW);
  	digitalWrite(RIGHT_FR, LOW);
  	analogWrite(LEFT_PWM, 0);
  	analogWrite(RIGHT_PWM, 0);


  	nh.initNode();
  	nh.subscribe(SubLeft);
  	nh.subscribe(SubRight);

	LeftWheelPidController.SetOutputLimits(MinPwm,MaxPwm);
	RightWheelPidController.SetOutputLimits(MinPwm,MaxPwm);

	//turn PID controllers on
	LeftWheelPidController.SetMode(AUTOMATIC);
	RightWheelPidController.SetMode(AUTOMATIC);
}

ISR(TIMER1_COMPA_vect) // 0.5s timer
{
	//this timer is needed to run the 0.25s timer for whatever reason
}

ISR(TIMER1_COMPB_vect) // 0.25s timer
{
	double leftSpeed = 60 * (LeftTick / (float)PulsesPerRevolution) / 0.25; //rpm
	double rightSpeed = 60 * (RightTick / (float)PulsesPerRevolution) /0.25; // rpm

	RightTick = 0;
	LeftTick = 0;

	LeftWheelState = map(leftSpeed ,0,MaxRpm,MinPwm,MaxPwm);
	RightWheelState = map(rightSpeed ,0,MaxRpm,MinPwm,MaxPwm);
}


void loop()
{
	nh.spinOnce();
	LeftWheelPidController.Compute();
	RightWheelPidController.Compute();
	WriteControlEffort(LEFT_PWM, LEFT_FR, LeftWheelControlEffort, LeftWheelSetPoint, LeftWheelBackward);
	WriteControlEffort(RIGHT_PWM, RIGHT_FR, RightWheelControlEffort, RightWheelSetPoint, RightWheelBackward);
}

void leftEncoder()
{
	LeftTick += 1;
}
void rightEncoder() 
{
	RightTick += 1;
}

void WriteControlEffort(unsigned int pwmPin, unsigned int frPin, double controlEffort, double setPoint, bool backward) 
{
	unsigned int pwm = backward ? MaxPwm - controlEffort : controlEffort;

	if (!backward && setPoint <= MinPwm)
	{
		digitalWrite(frPin,HIGH);
		digitalWrite(pwmPin,HIGH);
	}
	else
	{
		if (backward)
		{
			digitalWrite(frPin,HIGH);
		}
		else 
		{
			digitalWrite(frPin,LOW);
		}

		analogWrite(pwmPin, pwm);
	}
}

