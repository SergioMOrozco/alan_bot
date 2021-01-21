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

//Prototypes
void leftWheelControlEffortCallback(const std_msgs::Float64&);
void rightWheelControlEffortCallback(const std_msgs::Float64&);

unsigned long current_t = 0, prev_t = 0;

//PID variables
//float Kp = 0.6, Ki = .01 , Kd = 10;
//float Kp = 0.6, Ki = .008 , Kd = 100;
float Kp = 0.3, Ki =0.01 , Kd = 1000;

double LeftWheelState = 0, LeftWheelControlEffort = 0, LeftWheelSetPoint = 0;
PID leftWheelPidController (&LeftWheelState, &LeftWheelControlEffort, &LeftWheelSetPoint, Kp, Ki, Kd, DIRECT); 

ros::NodeHandle nh;

unsigned int left_tick = 0;
unsigned int right_tick = 0;


//motors
int MaxRpm = 180; // max rpm at 6v
int PulsesPerRevolution = 384;
int desiredPwm = 180;
int minPwm = 115; //pwm where motor stops moving at 6v
int maxPwm = 255;

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

	leftWheelPidController.SetOutputLimits(minPwm,maxPwm);

	//turn PID Controllers on
	leftWheelPidController.SetMode(AUTOMATIC);
}

ISR(TIMER1_COMPA_vect) // 0.5s timer
{

	if (true) {
		return;
	}

	double left_speed = 60 * (left_tick / (float)PulsesPerRevolution) / 0.5; //rpm
	double right_speed = 60 * (right_tick / (float)PulsesPerRevolution) /0.5; // rpm


	//right_tick = 0;
	left_tick = 0;

	LeftWheelState = map(left_speed,0,MaxRpm,minPwm,maxPwm);

	String lol = String("Left Tick: " + String(LeftWheelState));
	char* lol_c = lol.c_str();
	nh.loginfo(lol_c);
}

ISR(TIMER1_COMPB_vect) // 0.25s timer
{
	if (false) {
		return;
	}

	double left_speed = 60 * (left_tick / (float)PulsesPerRevolution) / 0.25; //rpm
	double right_speed = 60 * (right_tick / (float)PulsesPerRevolution) /0.25; // rpm


	//right_tick = 0;
	left_tick = 0;

	LeftWheelState = map(left_speed,0,MaxRpm,minPwm,maxPwm);

	String lol = String("Left Tick: " + String(LeftWheelState));
	char* lol_c = lol.c_str();
	nh.loginfo(lol_c);
}

void loop() {
	nh.spinOnce();

	LeftWheelSetPoint = desiredPwm;

	leftWheelPidController.Compute();



	if (LeftWheelControlEffort == minPwm) {
		digitalWrite(LEFT_FR,HIGH);
		digitalWrite(LEFT_PWM,HIGH);
	}
	else {
		digitalWrite(LEFT_FR,LOW);
		analogWrite(LEFT_PWM, LeftWheelControlEffort);
		//digitalWrite(LEFT_FR,HIGH);
		//digitalWrite(LEFT_PWM,HIGH);
	}
}

void left_encoder()
{
	left_tick += 1;
}
void right_encoder() 
{
	right_tick += 1;
}
