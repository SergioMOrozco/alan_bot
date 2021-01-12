#include <ros.h>
#include "Arduino.h"

#include <std_msgs/Float32.h>

ros::NodeHandle nh;

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif


// wheels
#define M_LEFT_PWM 6
#define M_LEFT_FR 7
#define M_RIGHT_PWM 5
#define M_RIGHT_FR 4

//encoders
#define RIGHT_ENCODER 2
#define LEFT_ENCODER 3

unsigned long left_tick = 0;
unsigned long right_tick = 0;


void turnWheel(const std_msgs::Float32 &wheel_power, unsigned int pwm_pin, unsigned int fr_pin) 
{
	//factor must be between -1 and 1
	float factor = max(min(wheel_power.data,1.0f),-1.0f);

	if (factor > 0)
	{
		nh.loginfo("Factor >= 0");
		
		digitalWrite(fr_pin, LOW);
		digitalWrite(pwm_pin, HIGH);
		//analogWrite(pwm_pin, (unsigned int)(255 * factor));
	}
	else if (factor < 0)
	{
		nh.loginfo("Factor <= 0");
		digitalWrite(fr_pin, HIGH);
		digitalWrite(pwm_pin, LOW);
		//analogWrite(pwm_pin, (unsigned int)(255 * (1.0f + factor)));
	}
	else {
		digitalWrite(fr_pin, HIGH);
		digitalWrite(pwm_pin, HIGH);
	}
}

void rightWheelCb( const std_msgs::Float32 &wheel_power)
{
	String message = "Wheel Power - Right" + String(wheel_power.data);
	char *message_convert = message.c_str();
	nh.loginfo(message_convert);
	turnWheel(wheel_power,M_RIGHT_PWM,M_RIGHT_FR);
}
void leftWheelCb( const std_msgs::Float32 &wheel_power)
{
	String message = "Wheel Power - Left" + String(wheel_power.data);
	char *message_convert = message.c_str();
	nh.loginfo(message_convert);
	turnWheel(wheel_power,M_LEFT_PWM,M_LEFT_FR);
}

ros::Subscriber<std_msgs::Float32> sub_right("wheel_power_right", &rightWheelCb);
ros::Subscriber<std_msgs::Float32> sub_left("wheel_power_left", &leftWheelCb);
void setup() 
{
  	// initialize LED digital pin as an output.
  	pinMode(LED_BUILTIN, OUTPUT);
  	digitalWrite(LED_BUILTIN, HIGH);


	// encoders
	
	// hall effect sensor is an open drain output. This means that the
	// sensor will only pull line low, but it won't pull it high. It will just
	// leave it floating, so we need to pull it up to a voltage
	pinMode(LEFT_ENCODER,INPUT_PULLUP);
	pinMode(RIGHT_ENCODER,INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER),left_encoder,CHANGE);
	attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER),right_encoder,CHANGE);


	// wheels
  	pinMode(M_LEFT_PWM, OUTPUT);
  	pinMode(M_LEFT_FR, OUTPUT);
  	pinMode(M_RIGHT_PWM, OUTPUT);
  	pinMode(M_RIGHT_FR, OUTPUT);

  	// Init motors to stop
  	digitalWrite(M_LEFT_FR, LOW);
  	digitalWrite(M_RIGHT_FR, LOW);
  	analogWrite(M_LEFT_PWM, 0);
  	analogWrite(M_RIGHT_PWM, 0);

  	nh.initNode();
  	nh.subscribe(sub_right);
  	nh.subscribe(sub_left);
}

void loop() {
	nh.spinOnce();

	String right_message = "Right Tick: " + String(right_tick);
	char *right_message_convert = right_message.c_str();
	String left_message = "Left Tick: " + String(left_tick);
	char *left_message_convert = left_message.c_str();

	nh.loginfo(right_message_convert);
	nh.loginfo(left_message_convert);



}

void left_encoder() {
	left_tick += 1;
}
void right_encoder() {
	right_tick += 1;
}
