#include <Arduino.h>

#define PROBOT_WIFI_AP_PASSWORD "alfa8084"


#include <probot.h>
#include <probot/io/joystick_api.hpp>
//#include <probot/command.hpp>
#include <MecanumDrive.h>

#include <PCA9685.h>

//using Scheduler = probot::command::Scheduler;

#define PWM_FREQUENCY 200.0f
#define PWM_PERIOD 1000000.0f/PWM_FREQUENCY	// In microseconds

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
	REAR_RIGHT_FWD_PWM, REAR_RIGHT_BWD_PWM,
	50, 
	0.5f, 0.5f
);

#define INTAKE_INWARD_PWM 9
#define INTAKE_OUTWARD_PWM 10

#define ELEVATOR0_UPWARD_PWM 11
#define ELEVATOR0_DOWNWARD_PWM 12
#define ELEVATOR1_UPWARD_PWM 14
#define ELEVATOR1_DOWNWARD_PWM 13

#define GRABBER_SERVO_PWM 15
#define ARM_SERVO_PWM 16 


// Datasheet PWM specs for the DS3225mg servo
#define SERVO_PULSE_WIDTH_MIN 500.0f	// In microseconds
#define SERVO_PULSE_WIDTH_MAX 2500.0f	// In microseconds
#define SERVO_PULSE_WIDTH_NEUTRAL 1500.0f	// In microseconds
#define SERVO_PULSE_WIDTH_STEP (SERVO_PULSE_WIDTH_MAX-SERVO_PULSE_WIDTH_MIN)/180.0f	// In microseconds per degree
#define SERVO_MIN_ANGLE 0.0f	// In degrees
#define SERVO_MAX_ANGLE 180.0f	// In degrees


void setupPwmController(PCA9685& controller){
	controller.resetDevices();
	controller.init();
	controller.setPWMFrequency(PWM_FREQUENCY);
}

/**
 * Function signiture to be fed to MecanumDrive.drive() 
 * method in order to use PCA9685 module as the PWM module
 * @param pin PCA9685 channel to set
 * @param pwm (0, 255)
 */
void setMotorPWM(int pin, int pwm){
	pwmModule.setChannelPWM(pin, map(constrain(pwm, 0, 255), 0, 255, 0, 4095));
}

/**
 * @param pin PCA9685 channel to set
 * @param angle in degrees(0, 180). Note that the servo starting 
 * position when powered on is 90 degrees
 */
void setServoAngle(int pin, float angle){
	float pulseWidth = constrain(angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE)*SERVO_PULSE_WIDTH_STEP + SERVO_PULSE_WIDTH_MIN;
	uint16_t pwm = uint16_t((pulseWidth * 4096) / PWM_PERIOD);
	pwm = constrain(pwm, 0, 4095);

	pwmModule.setChannelPWM(pin, pwm);
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

	if(js.getRawButton(1)) setMotorPWM(INTAKE_INWARD_PWM, 100);
	else setMotorPWM(INTAKE_INWARD_PWM, 0);

	if(js.getRawButton(2)) {
		setMotorPWM(ELEVATOR0_UPWARD_PWM, 100);
		setMotorPWM(ELEVATOR1_UPWARD_PWM, 100);
	}
	else{
		setMotorPWM(ELEVATOR0_UPWARD_PWM, 0);
		setMotorPWM(ELEVATOR1_UPWARD_PWM, 0);
	}
	if(js.getRawButton(3)) {
		setMotorPWM(ELEVATOR0_DOWNWARD_PWM, 100);
		setMotorPWM(ELEVATOR1_DOWNWARD_PWM, 100);
	}
	else{
		setMotorPWM(ELEVATOR0_DOWNWARD_PWM, 0);
		setMotorPWM(ELEVATOR1_DOWNWARD_PWM, 0);
	}

	if(js.getRawButton(4)) setServoAngle(GRABBER_SERVO_PWM, 140);
	else setServoAngle(GRABBER_SERVO_PWM, 90);
	
	if(js.getRawButton(5)) setServoAngle(ARM_SERVO_PWM, 160);
	else setServoAngle(ARM_SERVO_PWM, 90);


	delay(20);
}

void autonomousInit(){
	Serial.println("Autononous started");
}

void autonomousLoop(){

}