#include <ros.h>
#include "Arduino.h"
#include <PID_v1.h>
#include <std_msgs/Float32.h>
#include <stdlib.h>

//ROS variables
ros::NodeHandle nh;

//encoder variables
unsigned long left_tick = 0;
unsigned long right_tick = 0;


// PID variables
double Kp= 10, Ki = 10 , Kd = 10;
double ActualLeftPwm = 0, CorrectedLeftPwm = 0, DesiredLeftPwm = 0;
double ActualRightPwm = 0, CorrectedRightPwm = 0, DesiredRightPwm = 0;
PID leftWheelPidController (&ActualLeftPwm, &CorrectedLeftPwm, &DesiredLeftPwm, Kp, Ki, Kd, DIRECT); 
PID rightWheelPidController(&ActualRightPwm, &CorrectedRightPwm, &DesiredRightPwm, Kp, Ki, Kd, DIRECT); 

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif


// wheels
#define LEFT_PWM 6
#define LEFT_FR 7
#define RIGHT_PWM 5
#define RIGHT_FR 4

bool LeftDirectionForward = true, RightDirectionForward = true, Stop = true;

//encoders
#define RIGHT_ENCODER 2
#define LEFT_ENCODER 3

//motors
int MaxRpm = 230;
double PulsesPerRevolution = 384.00;



void turnWheel(const std_msgs::Float32 &wheel_power, unsigned int pwm_pin, unsigned int fr_pin) 
{
	//factor must be between -1 and 1
	float factor = max(min(wheel_power.data,1.0f),-1.0f);
	Stop = (factor == 0);

	//We want to move forward
	if (factor > 0)
	{
		
		nh.loginfo("Factor >= 0");
		
		digitalWrite(fr_pin, LOW);
		analogWrite(pwm_pin, (unsigned int)(255 * factor));
	}

	// We want to move backward
	else if (factor < 0)
	{
		nh.loginfo("Factor <= 0");
		digitalWrite(fr_pin, HIGH);
		analogWrite(pwm_pin, (unsigned int)(255 * (1.0f + factor))); // PID will be reversed (0 is fastest, 255 is stopped)
	}

	//We want to stop
	else {
		digitalWrite(fr_pin, HIGH);
		digitalWrite(pwm_pin, HIGH);
	}

	// Set our desired PWM that the PID Controller will work towards
	unsigned int pwm = (unsigned int)(255 * abs(factor));

	if ( pwm_pin == LEFT_PWM)
	{
		DesiredLeftPwm = pwm; 
	}
	else 
	{
		DesiredRightPwm = pwm; 
	}
}

void rightWheelCb( const std_msgs::Float32 &wheel_power)
{
	String message = "Wheel Power - Right" + String(wheel_power.data);
	char *message_convert = message.c_str();
	nh.loginfo(message_convert);
	turnWheel(wheel_power,RIGHT_PWM,RIGHT_FR);
	RightDirectionForward = wheel_power.data >= 0;
}
void leftWheelCb( const std_msgs::Float32 &wheel_power)
{
	String message = "Wheel Power - Left" + String(wheel_power.data);
	char *message_convert = message.c_str();
	nh.loginfo(message_convert);
	turnWheel(wheel_power,LEFT_PWM,LEFT_FR);
	LeftDirectionForward = wheel_power.data >= 0;
}

ros::Subscriber<std_msgs::Float32> sub_right("wheel_power_right", &rightWheelCb);
ros::Subscriber<std_msgs::Float32> sub_left("wheel_power_left", &leftWheelCb);

std_msgs::Float32 actual_right_pwm;
std_msgs::Float32 desired_right_pwm;

ros::Publisher pub_actual_right_pwm("actual_right_pwm", &actual_right_pwm);
ros::Publisher pub_desired_right_pwm("desired_right_pwm", &desired_right_pwm);

void setup() 
{
	//Serial.begin(115200);

  	// initialize LED digital pin as an output.
  	pinMode(LED_BUILTIN, OUTPUT);
  	digitalWrite(LED_BUILTIN, HIGH);


	// Setup Timer
	// Reference: https://www.youtube.com/watch?v=IdL0_ZJ7V2s&t=110s
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1B |= (1<<CS10); //prescaler 1
	TIMSK1 |= (1<<TOIE1); //enable timer overflow

	// encoders
	
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
  	nh.subscribe(sub_right);
  	nh.subscribe(sub_left);
	nh.advertise(pub_actual_right_pwm);
	nh.advertise(pub_desired_right_pwm);


	//turn PID controllers on
	leftWheelPidController.SetMode(AUTOMATIC);
	rightWheelPidController.SetMode(AUTOMATIC);
}

ISR(TIMER1_OVF_vect) // 4ms timer
{
	// no need for PID calculations on stop
	if (Stop)
	{
		return;
	}
	double left_speed = 60000.0 * (left_tick / PulsesPerRevolution) / 4.0; //rpm
	double right_speed = 60000.0 * (right_tick / PulsesPerRevolution) / 4.0; // rpm


	left_tick = 0;
	right_tick = 0;

	ActualLeftPwm = map(left_speed,0,MaxRpm,0,255);
	ActualRightPwm = map(right_speed,0,MaxRpm,0,255);

	// send over PWM desired and actual values for calibration
	actual_right_pwm.data = ActualRightPwm;
	//pub_actual_right_pwm.publish(&actual_right_pwm);

	desired_right_pwm.data = DesiredRightPwm;
	//pub_desired_right_pwm.publish(&desired_right_pwm);

	leftWheelPidController.Compute();
	rightWheelPidController.Compute();

	WriteCorrectedPwms();
}

void loop()
{
	nh.spinOnce();
}

void left_encoder()
{
	left_tick += 1;
}
void right_encoder() 
{
	right_tick += 1;
}

void WriteCorrectedPwms() 
{
	unsigned int leftPwm = LeftDirectionForward ? CorrectedLeftPwm : 255 - CorrectedLeftPwm;
	unsigned int rightPwm = RightDirectionForward ? CorrectedRightPwm : 255 - CorrectedRightPwm;

	analogWrite(LEFT_PWM, leftPwm);
	analogWrite(RIGHT_PWM, rightPwm);
}
