#include <ros.h>
#include "Arduino.h"

#include <std_msgs/Float32.h>

ros::NodeHandle nh;

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#define M_LEFT_PWM 6
#define M_LEFT_FR 7
#define M_RIGHT_PWM 5
#define M_RIGHT_FR 4

void turnWheel(const std_msgs::Float32 &wheel_power, unsigned int pwm_pin, unsigned int fr_pin) 
{
	// stop motor briefly
	//digitalWrite(fr_pin, LOW);
	//digitalWrite(pwm_pin, LOW);
	//delay(20);
	
	//factor must be between -1 and 1
	float factor = max(min(wheel_power.data,1.0f),-1.0f);

	if (factor > 0)
	{
		nh.loginfo("Factor >= 0");
		
		digitalWrite(fr_pin, LOW);
		analogWrite(pwm_pin, (unsigned int)(255 * factor));
	}
	else if (factor < 0)
	{
		nh.loginfo("Factor <= 0");
		digitalWrite(fr_pin, HIGH);
		analogWrite(pwm_pin, (unsigned int)(255 * (1.0f + factor)));
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
	//delay(1000);
}
