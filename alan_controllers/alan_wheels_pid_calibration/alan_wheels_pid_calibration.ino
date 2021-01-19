#include <ros.h>
#include "Arduino.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <stdlib.h>

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

//PID variables
std_msgs::Float64 LeftWheelState, LeftWheelControlEffort, LeftWheelSetPoint;
std_msgs::Float64 RightWheelState, RightWheelControlEffort, RightWheelSetPoint;

ros::NodeHandle nh;
ros::Publisher LeftWheelSetPointPublisher("setpoint",&LeftWheelSetPoint);
ros::Publisher LeftWheelStatePublisher("state",&LeftWheelState);
ros::Subscriber<std_msgs::Float64> LeftWheelControlEffortSubscriber("control_effort",&leftWheelControlEffortCallback);

unsigned int left_tick = 0;
unsigned int right_tick = 0;


//motors
int MaxRpm = 85;
int PulsesPerRevolution = 384;


void leftWheelControlEffortCallback(const std_msgs::Float64& control_effort) {
	String lol = String("Control Effort" + String(control_effort.data));
	char* lol_c = lol.c_str();
	nh.loginfo(lol_c);
	analogWrite(LEFT_PWM,control_effort.data);
}

void setup() {
	Serial.begin(115200);

	// Setup Timer
	// Reference: https://www.youtube.com/watch?v=IdL0_ZJ7V2s&t=110s
	TCCR1A = 0;
	TCCR1B = 0;
	OCR1A = 31249;
	TCCR1B |= (1<<CS12) | (1<<WGM12); //prescaler 256, CTCMode
	TIMSK1 |= (1<<OCIE1A); //enable timer overflow

	//specifically for pin 5 and 6: Reference - https://www.etechnophiles.com/change-frequency-pwm-pins-arduino-uno/
	TCCR0B = TCCR0B & B11111000 | B00000010; // for PWM frequency of 7812.50 Hz

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
	nh.advertise(LeftWheelSetPointPublisher);
	nh.advertise(LeftWheelStatePublisher);
	nh.subscribe(LeftWheelControlEffortSubscriber);
}

ISR(TIMER1_COMPA_vect) // 0.5s timer
{
	double left_speed = 60.0 * (left_tick / (float)PulsesPerRevolution) / 0.5; //rpm
	double right_speed = 60.0 * (right_tick / (float)PulsesPerRevolution) / 0.5; // rpm

	if (left_speed > MaxRpm || right_speed > MaxRpm) 
	{
		nh.logwarn("Calculating RPM greater than max RPM. Adjust Max RPM.");
	}

	left_tick = 0;
	right_tick = 0;

	LeftWheelState.data = map(left_speed,0,MaxRpm,0,255);
	RightWheelState.data = map(right_speed,0,MaxRpm,0,255);

	String lol = String("State: " + String(LeftWheelState.data));
	char* lol_c = lol.c_str();
	nh.loginfo(lol_c);
	
	LeftWheelStatePublisher.publish(&LeftWheelState);
}

void loop() {
	nh.spinOnce();

	LeftWheelSetPoint.data = 128;
	RightWheelSetPoint.data = 128;

	LeftWheelSetPointPublisher.publish(&LeftWheelSetPoint);
	delay(1000);
}

void left_encoder()
{
	left_tick += 1;
}
void right_encoder() 
{
	right_tick += 1;
}
