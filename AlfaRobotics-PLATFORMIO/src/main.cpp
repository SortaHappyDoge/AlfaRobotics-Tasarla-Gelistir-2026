#include <Arduino.h>

#define PROBOT_WIFI_AP_PASSWORD "test12345"


#include <probot.h>
#include <probot/io/joystick_api.hpp>
//#include <probot/command.hpp>
#include <MecanumDrive.h>

#include <PCA9685.h>

//using Scheduler = probot::command::Scheduler;

PCA9685 pwmModule;

#define FRONT_LEFT_FWD_PWM 1
#define FRONT_LEFT_BWD_PWM 2
#define FRONT_RIGHT_FWD_PWM 3
#define FRONT_RIGHT_BWD_PWM 4
#define REAR_LEFT_FWD_PWM 5
#define REAR_LEFT_BWD_PWM 6
#define REAR_RIGHT_FWD_PWM 7
#define REAR_RIGHT_BWD_PWM 8

MecanumDrive m_mecanumDrive(
	FRONT_LEFT_FWD_PWM, FRONT_LEFT_BWD_PWM,
	FRONT_RIGHT_FWD_PWM, FRONT_RIGHT_BWD_PWM,
	REAR_LEFT_FWD_PWM, REAR_LEFT_BWD_PWM,
	REAR_RIGHT_FWD_PWM, REAR_RIGHT_FWD_PWM,
	50, 
	50, 50
);

#define INTAKE_INWARD_PWM 9
#define INTAKE_OUTWARD_PWM 10

#define ELEVATOR0_UPWARD_PWM 11
#define ELEVATOR0_DOWNWARD_PWM 12
#define ELEVATOR1_UPWARD_PWM 14
#define ELEVATOR1_DOWNWARD_PWM 13

#define GRABBER_SERVO_PWM 15
#define ARM_SERVO_PWM 16 


void setupPwmController(PCA9685& controller){
	controller.resetDevices();
	controller.init();
	controller.setPWMFrequency(200);
}

void setMotorPWM(int pin, int pwm){
	pwmModule.setChannelPWM(pin, map(pwm, 0, 255, 0, 4096));
}

void robotInit(){
	Serial.begin(115200);
	Wire.begin();

	setupPwmController(pwmModule);
}

void robotEnd(){

}

void teleopInit(){
	
}

void teleopLoop(){
	auto js = probot::io::joystick_api::makeDefault();

	float joystickX = js.getLeftY(); float joystickY = js.getLeftX(); float joystickOmega = js.getRightX();
	m_mecanumDrive.drive(joystickX, joystickY, joystickOmega, setMotorPWM);

	delay(20);
}

void autonomousInit(){
	Serial.println("Autononous started");
}

void autonomousLoop(){

}