#include <ros.h>
#include "Arduino.h"
#include <PID_v1.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <stdlib.h>

//ROS variables : need to decrease publishing rate or increase bufffer: reference - https://github.com/tonybaltovski/ros_arduino/issues/10
ros::NodeHandle nh;
//encoder variables
int left_tick = 0;
int right_tick = 0;

// wheels
#define LEFT_PWM 6
#define LEFT_FR 7
#define RIGHT_PWM 5
#define RIGHT_FR 4

//encoders
#define RIGHT_ENCODER 2
#define LEFT_ENCODER 3


bool Stopped = true;

// PID variables
float Kp = 0, Ki = 0 , Kd = 0;
double ActualLeftPwm = 0, CorrectedLeftPwm = 0, DesiredLeftPwm = 0;
double ActualRightPwm = 0, CorrectedRightPwm = 0, DesiredRightPwm = 0;

PID leftWheelPidController (&ActualLeftPwm, &CorrectedLeftPwm, &DesiredLeftPwm, Kp, Ki, Kd, DIRECT); 
PID rightWheelPidController(&ActualRightPwm, &CorrectedRightPwm, &DesiredRightPwm, Kp, Ki, Kd, DIRECT); 

std_msgs::Float32 actual_right_pwm;
std_msgs::Float32 desired_right_pwm;
std_msgs::Float32 actual_left_pwm;
std_msgs::Float32 desired_left_pwm;

ros::Publisher pub_actual_right_pwm("actual_right_pwm", &actual_right_pwm);
ros::Publisher pub_desired_right_pwm("desired_right_pwm", &desired_right_pwm);
ros::Publisher pub_actual_left_pwm("actual_left_pwm", &actual_left_pwm);
ros::Publisher pub_desired_left_pwm("desired_left_pwm", &desired_left_pwm);

void pidCalibrationCb( const std_msgs::Float32MultiArray &pid_calibration)
{

	int length = sizeof(pid_calibration.data) / sizeof(pid_calibration.data[0]);

	if ( length == 3) 
	{
		String message = "Kp: " + String(pid_calibration.data[0]);
		char *message_convert = message.c_str();
		nh.loginfo(message_convert);

		message = "Ki: " + String(pid_calibration.data[1]);
		*message_convert = message.c_str();
		nh.loginfo(message_convert);

		message = "Kd: " + String(pid_calibration.data[2]);
		*message_convert = message.c_str();
		nh.loginfo(message_convert);

		Kp = pid_calibration.data[0];
		Ki = pid_calibration.data[1];
		Kd = pid_calibration.data[2];
		leftWheelPidController.SetTunings(Kp,Ki,Kd);
		rightWheelPidController.SetTunings(Kp,Ki,Kd);
	}
}
void stopMotorsCb( const std_msgs::Float32 &stop_motors)
{
	Stopped = stop_motors.data == 0;
}

ros::Subscriber<std_msgs::Float32MultiArray> pid_calibration("pid_calibration", &pidCalibrationCb);
ros::Subscriber<std_msgs::Float32> stop_motors("stop_motors", &stopMotorsCb);

//motors
int MaxRpm = 230;
int PulsesPerRevolution = 384;


void setup() {
	// Setup Timer
	// Reference: https://www.youtube.com/watch?v=IdL0_ZJ7V2s&t=110s
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1B |= (1<<CS10); //prescaler 1
	TIMSK1 |= (1<<TOIE1); //enable timer overflow

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
	nh.advertise(pub_actual_right_pwm);
	nh.advertise(pub_desired_right_pwm);
	nh.advertise(pub_actual_left_pwm);
	nh.advertise(pub_desired_left_pwm);
	nh.subscribe(pid_calibration);
	nh.subscribe(stop_motors);


	//turn PID controllers on
	leftWheelPidController.SetMode(AUTOMATIC);
	rightWheelPidController.SetMode(AUTOMATIC);
}

ISR(TIMER1_OVF_vect) // 4ms timer
{
	if (Stopped) {
		return;
	}
	double left_speed = 60000.0 * (left_tick / (float)PulsesPerRevolution) / 4.0; //rpm
	double right_speed = 60000.0 * (right_tick / (float)PulsesPerRevolution) / 4.0; // rpm

	left_tick = 0;
	right_tick = 0;

	ActualLeftPwm = map(left_speed,0,MaxRpm,0,255);
	ActualRightPwm = map(right_speed,0,MaxRpm,0,255);

	leftWheelPidController.Compute();
	rightWheelPidController.Compute();

	WriteCorrectedPwms();
}

void loop() {
	nh.spinOnce();

	// send over PWM desired and actual values for calibration
	actual_right_pwm.data = ActualRightPwm;
	pub_actual_right_pwm.publish(&actual_right_pwm);

	desired_right_pwm.data = DesiredRightPwm;
	pub_desired_right_pwm.publish(&desired_right_pwm);

	actual_left_pwm.data = ActualLeftPwm;
	pub_actual_left_pwm.publish(&actual_left_pwm);

	desired_left_pwm.data = DesiredLeftPwm;
	pub_desired_left_pwm.publish(&desired_left_pwm);

	if (!Stopped) 
	{
		//move forward for one second
		DesiredLeftPwm = 255;
		digitalWrite(LEFT_FR, LOW);
		analogWrite(LEFT_PWM, DesiredLeftPwm);

		DesiredRightPwm = 255;
		digitalWrite(RIGHT_FR, LOW);
		analogWrite(RIGHT_PWM, DesiredRightPwm);
	}
	else {
  		digitalWrite(LEFT_FR, HIGH);
  		digitalWrite(RIGHT_FR, HIGH);
  		digitalWrite(LEFT_PWM, HIGH);
  		digitalWrite(RIGHT_PWM, HIGH);
		ActualLeftPwm = 0; 
		ActualRightPwm = 0;
 
	}
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

void WriteCorrectedPwms() 
{
	//unsigned int leftPwm = LeftDirectionForward ? CorrectedLeftPwm : 255 - CorrectedLeftPwm;
	//unsigned int rightPwm = RightDirectionForward ? CorrectedRightPwm : 255 - CorrectedRightPwm;

	analogWrite(LEFT_PWM, CorrectedLeftPwm);
	analogWrite(RIGHT_PWM, CorrectedRightPwm);
}
