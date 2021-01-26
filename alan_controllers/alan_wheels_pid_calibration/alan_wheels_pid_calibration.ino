#include <ros.h>
#include "Arduino.h"
#include <PID_v1.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>
#include <PID_v1.h>

// wheels
#define LEFT_PWM 6
#define LEFT_FR 7
#define RIGHT_PWM 5
#define RIGHT_FR 4

//encoders
#define RIGHT_ENCODER 2
#define LEFT_ENCODER 3

//prototypes 
void left_encoder();
void right_encoder();

//PID variables
float Kp = 0.3, Ki =0.01 , Kd = 1000; //original

double LeftWheelState = 0, LeftWheelControlEffort = 0, LeftWheelSetPoint = 0;
double RightWheelState = 0, RightWheelControlEffort = 0, RightWheelSetPoint = 0;
PID RightWheelPidController (&RightWheelState, &RightWheelControlEffort, &RightWheelSetPoint, Kp, Ki, Kd, DIRECT); 
PID LeftWheelPidController (&LeftWheelState, &LeftWheelControlEffort, &LeftWheelSetPoint, Kp, Ki, Kd, DIRECT); 

ros::NodeHandle nh;

unsigned int LeftTick = 0;
unsigned int RightTick = 0;


//motors
int DesiredPwm = 240;
int MinPwm = 115; //pwm where motor stops moving at 6v
int MaxPwm = 255;
//int MaxRpm = 180; // max rpm at 6v
int MaxRpm = 96; // max rpm at 6v with current load
int PulsesPerRevolution = 384;

void setup() {
	Serial.begin(115200);

	// Setup Timer
	// Reference: https://www.youtube.com/watch?v=IdL0_ZJ7V2s&t=110s
	TCCR1A = 0;
	TCCR1B = 0;
	OCR1A = 31249;
	OCR1B = 15624;
	TCCR1B |= (1<<CS12) | (1<<WGM12);
	TIMSK1 |= (1<<OCIE1A) | (1<<OCIE1B); //enable timer overflow

	//specifically for pin 5 and 6: Reference - https://www.etechnophiles.com/change-frequency-pwm-pins-arduino-uno/
	TCCR0B = TCCR0B & B11111000 | B00000001; // for PWM frequency of 62500.00 Hz

	// hall effect sensor is an open drain output. This means that the
	// sensor will only pull line low, but it won't pull it high. It will just
	// leave it floating, so we need to pull it up to a voltage
	pinMode(LEFT_ENCODER,INPUT_PULLUP);
	pinMode(RIGHT_ENCODER,INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER),left_encoder,RISING);
	attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER),right_encoder,RISING);


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

	LeftWheelPidController.SetOutputLimits(MinPwm,MaxPwm);
	RightWheelPidController.SetOutputLimits(MinPwm,MaxPwm);

	//turn PID Controllers on
	LeftWheelPidController.SetMode(AUTOMATIC);
	RightWheelPidController.SetMode(AUTOMATIC);

	LeftWheelSetPoint = DesiredPwm;
	RightWheelSetPoint = DesiredPwm;
}

ISR(TIMER1_COMPA_vect) // 0.5s timer
{
	String leftStateLog = String("Left State: " + String(LeftWheelState));
	char* leftStateLogConverted = leftStateLog.c_str();
	nh.loginfo(leftStateLogConverted);

	String rightStateLog = String("Right State: " + String(RightWheelState));
	char* rightStateLogConverted = rightStateLog.c_str();
	nh.loginfo(rightStateLogConverted);
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

void loop() {
	nh.spinOnce();

	LeftWheelPidController.Compute();
	RightWheelPidController.Compute();

	// may not need to set FR and PWM to high
	if (LeftWheelControlEffort == MinPwm) {
		digitalWrite(LEFT_FR,HIGH);
		digitalWrite(LEFT_PWM,HIGH);
	}
	else {
		digitalWrite(LEFT_FR,LOW);
		analogWrite(LEFT_PWM, LeftWheelControlEffort);
	}

	if (RightWheelControlEffort == MinPwm) {
		digitalWrite(RIGHT_FR,HIGH);
		digitalWrite(RIGHT_PWM,HIGH);
	}
	else {
		digitalWrite(RIGHT_FR,LOW);
		analogWrite(RIGHT_PWM, RightWheelControlEffort);
	}
}

void left_encoder()
{
	LeftTick += 1;
}
void right_encoder() 
{
	RightTick += 1;
}
