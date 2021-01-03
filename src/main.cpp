#include "main.h"
using namespace pros;

/**
* A callback function for LLEMU's center button.
*
* When this callback is fired, it will toggle line 2 of thea LCD text between
* "I was pressed!" and nothing.
*/
Controller master(E_CONTROLLER_MASTER);
Controller partner(E_CONTROLLER_PARTNER);

Motor leftBack(17, MOTOR_GEARSET_18, false);
Motor leftFront(12, MOTOR_GEARSET_18, false);
Motor rightFront(15, MOTOR_GEARSET_18, true);
Motor rightBack(18, MOTOR_GEARSET_18, true);
Motor leftIntake(6, MOTOR_GEARSET_18, false);
Motor rightIntake(1, MOTOR_GEARSET_18, true);
Motor frontRoller(7, MOTOR_GEARSET_18, false);
Motor backRoller(9, MOTOR_GEARSET_18, false);
Imu inert(4);
Optical vision(3);

void on_center_button() {
 static bool pressed = false;
 pressed = !pressed;
 if (pressed) {
   pros::lcd::set_text(2, "I was pressed!");
 } else {
   pros::lcd::clear_line(2);
 }
}

/**
* Runs initialization code. This occurs as soon as the program is started.
*
* All other competition modes are blocked by initialize; it is recommended
* to keep execution time for this mode under a few seconds.
*/

void initialize() {
 pros::lcd::initialize();
 pros::lcd::set_text(1, "Hello PROS User!");

 pros::lcd::register_btn1_cb(on_center_button);
}

int sign(int num) {
  if(num > 0 ) {
    return 1;
  }
  else {
    return -1;
  }
}
void resetPosition(){
 leftFront.tare_position();
 rightFront.tare_position();
 leftBack.tare_position();
 rightBack.tare_position();
 leftIntake.tare_position();
 rightIntake.tare_position();
 frontRoller.tare_position();
 backRoller.tare_position();
}

// Returns speed the robot should be travelling at
int moveSpeed(int target){
 double travelled;
 double pwr = 0;
 int minSpeed = 40;
 int accelDistance = 150;
 int deccelDistance = 500;

 // Find ticks needed to reach target
 double error = sign(target)*(abs(target) - fabs(leftFront.get_position()));
 travelled = leftFront.get_position();

 // Increase speed gradually, run at top speed, then gradually decrease speed
 if(fabs(travelled) <= accelDistance && fabs(travelled) <= abs(target)/2){
   pwr = 127.0/accelDistance*travelled;
 }
 else if(fabs(error) <= deccelDistance && fabs(error) <= abs(target)/2){
   pwr = 127.0/deccelDistance*error;
 }
 else{
   pwr = sign(target)*127;
 }

 // Ensure that the bot is never too slow or not moving
 if(fabs(pwr)<minSpeed) {
   pwr = sign(target)*minSpeed;
 }
 return pwr;
}

// Find how off the robot is from its target heading
int headingCorrection(int targetHeading){
 double error = targetHeading - inert.get_rotation();
 printf("correction %f %f", error, inert.get_rotation());
 return error*3;
}

// Moves the robot using speeds from moveSpeed and headingCorrection
void motionController(int target, int targetHeading){
 targetHeading = targetHeading + 90;
 target = target * 5/3;
 resetPosition();
 int error = sign(target)*(abs(target) - fabs(leftFront.get_position()));
 while(sign(target)*(error) > 5){
   leftFront.move(moveSpeed(target) + headingCorrection(targetHeading));
   leftBack.move(moveSpeed(target) + headingCorrection(targetHeading));
   rightFront.move(moveSpeed(target) - headingCorrection(targetHeading));
   rightBack.move(moveSpeed(target) - headingCorrection(targetHeading));
   error = sign(target)*(abs(target) - fabs(leftFront.get_position()));
   printf("%f", inert.get_rotation());
 }
 leftFront.move(0);
 rightFront.move(0);
 leftBack.move(0);
 rightBack.move(0);
}

void turnPos(int angle){
double initialAngle = inert.get_rotation();
double currentAngle = inert.get_rotation() - initialAngle;
int error = angle - currentAngle;
double kp = 3;
while(fabs(currentAngle) < abs(angle)){
  leftFront.move( error*kp );
  leftBack.move( error*kp );
  rightFront.move( -error*kp );
  rightBack.move( -error*kp );
  error = angle - currentAngle;
  currentAngle = fabs(inert.get_rotation() - initialAngle);
}

leftFront.move(0);
rightFront.move(0);
leftBack.move(0);
rightBack.move(0);
}

void turnNeg(int angle){
 double initialAngle = inert.get_rotation();
 double currentAngle = inert.get_rotation() - initialAngle;
 int error = angle - currentAngle;
 double kp = 3;
 while(currentAngle > angle){
   leftFront.move( error*kp );
   leftBack.move( error*kp );
   rightFront.move( -error*kp );
   rightBack.move( -error*kp );
   error = angle - currentAngle;
   currentAngle = inert.get_rotation() - initialAngle;
 }
 leftFront.move(0);
 rightFront.move(0);
 leftBack.move(0);
 rightBack.move(0);
}

void bruteForce(int lspeed, int rspeed, int time) {
 leftFront.move(lspeed);
 leftBack.move(lspeed);
 rightFront.move(rspeed);
 rightBack.move(rspeed);
 delay(time);
 leftFront.move(0);
 leftBack.move(0);
 rightFront.move(0);
 rightBack.move(0);
}

void bruteForce(int lspeed, int rspeed) {
 leftFront.move(lspeed);
 leftBack.move(lspeed);
 rightFront.move(rspeed);
 rightBack.move(rspeed);
}

void slide(int speed, int time, int targetHeading) {
 leftFront.move(speed - headingCorrection(targetHeading));
 leftBack.move(-speed - headingCorrection(targetHeading));
 rightFront.move(-speed + headingCorrection(targetHeading));
 rightBack.move(speed + headingCorrection(targetHeading));
 delay(time);
 leftFront.move(0);
 leftBack.move(0);
 rightFront.move(0);
 rightBack.move(0);
}

void slideToTarget(int speed, int target, int targetHeading) {
 resetPosition();
 while(fabs(leftFront.get_position()) < target) {
   leftFront.move(speed + headingCorrection(targetHeading));
   leftBack.move(-speed + headingCorrection(targetHeading));
   rightFront.move(-speed - headingCorrection(targetHeading));
   rightBack.move(speed - headingCorrection(targetHeading));
 }
 leftFront.move(0);
 leftBack.move(0);
 rightFront.move(0);
 rightBack.move(0);
}

void pull() {
 leftIntake.move(127);
 rightIntake.move(127);
 frontRoller.move(0);
}

void intake() {
 leftIntake.move(127);
 rightIntake.move(127);
 frontRoller.move(60);
}

void outtake() {
 leftIntake.move(-127);
 rightIntake.move(-127);
 frontRoller.move(-80);
}

void openIntakes() {
 leftIntake.move(-127);
 rightIntake.move(-127);
 delay(700);
 leftIntake.move(0);
 rightIntake.move(0);
}

void stopIntake() {
 leftIntake.move(0);
 rightIntake.move(0);
 frontRoller.move(0);
}

void score( int n ) {
  resetPosition();
 frontRoller.move(127);
 delay(400);
 backRoller.move(-127);
 delay(400 * n);
 frontRoller.move(0);
 backRoller.move(127);
 delay(600);
 backRoller.move(0);
}

void slowScore(){
  stopIntake();
  frontRoller.move(90);
  delay(400);
  backRoller.move(-90);
  delay(700);
  frontRoller.move(0);
  backRoller.move(127);
  delay(600);
  backRoller.move(0);
}

void pullBack(int time) {
 backRoller.move(127);
 delay(time);
 backRoller.move(0);
}

/**
* Runs while the robot is in the disabled state of Field Management System or
* the VEX Competition Switch, following either autonomous or opcontrol. When
* the robot is enabled, this task will exit.
*/
void disabled() {}

/**
* Runs after initialize(), and before autonomous when connected to the Field
* Management System or the VEX Competition Switch. This is intended for
* competition-specific initialization routines, such as an autonomous selector
* on the LCD.
*
* This task will exit when the robot is enabled and autonomous or opcontrol
* starts.
*/
void competition_initialize() {}

/**
* Runs the user autonomous code. This function will be started in its own task
* with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the autonomous
* mode. Alternatively, this function may be called in initialize or opcontrol
* for non-competition testing purposes.
*
* If the robot is disabled or communications is lost, the autonomous task
* will be stopped. Re-enabling the robot will restart the task, not re-start it
* from where it left off.
*/
void MEPrint( int line, std::string a ){
  lcd::set_text(line, a);
}

void printNum( int line, int num ){
  char c[1024];
  sprintf(c, "%d", num);
  lcd::set_text(line, c);
}

void intake_blue_task_fn() {
  intake();
  while(!(vision.get_hue() > 150 && vision.get_hue() < 280)){
    delay(10);
  }
  stopIntake();
}

void spit_blue_task_fn() {
  MEPrint(2, "1");
  outtake();
  while(vision.get_hue() > 60){
    printNum(4, vision.get_hue());
    delay(10);
  }
  delay(300);
  MEPrint(2, "3");
  intake();
}

void spitBlue(){
  Task spit_blue_task(spit_blue_task_fn);
}

void intakeBlue(){
  Task intake_blue_task(intake_blue_task_fn);
}

void autonomous() {
 leftFront.set_brake_mode(E_MOTOR_BRAKE_HOLD);
 rightFront.set_brake_mode(E_MOTOR_BRAKE_HOLD);
 leftBack.set_brake_mode(E_MOTOR_BRAKE_HOLD);
 rightFront.set_brake_mode(E_MOTOR_BRAKE_HOLD);
 inert.reset();
 delay(2000);

 // Unfold
 bruteForce(-100, -100);
 backRoller.move(-127);
 frontRoller.move(127);
 delay(400);
 bruteForce(0, 0);
 backRoller.move(0);
 frontRoller.move(0);

 // Grab 2 more balls
 bruteForce(0, -127, 1000);
 intake();
 motionController(800, -65);
 turnPos(15);
 motionController(1000, -50);
 turnNeg(-35);
 motionController(300, -90);
 bruteForce(50, 50, 500);

 // left bottom
 motionController(-700, -90);
 intake();
 turnNeg(-45);
 motionController(900, -135);
 bruteForce(100, 100, 500);
 score(1);

 // left middle
 spitBlue();
 motionController(-1650, -135);
 turnPos(135);
 intake();
 motionController(1100, 0);
 turnNeg(-92);
 motionController(1100, -92);
 bruteForce(70, 70, 800);
 score(1);

 // Left top
 spitBlue();
 motionController(-750, -90);
 delay(500);
 intake();
 turnPos(90);
 motionController(1100, 0);
 bruteForce(80, 80, 800); // Grab another ball
 turnNeg(-65);
 motionController(700, -65);
 bruteForce(90, 90, 1000);
 score(1);

 // Middle top
 spitBlue();
 motionController(-1600, -45);
 turnPos(135);
 intakeBlue();
 motionController(950, 90);
 turnNeg(-90);
 motionController(1200, 0);
 bruteForce(70, 70, 800);
 score(2);

 // Right top
 spitBlue();
 motionController(-1200, 0);
 delay(500);
 turnPos(54);
 intake();
 motionController(2000, 54);
 bruteForce(100, 100, 500);
 score(2);

 // Right middle
 spitBlue();
 motionController(-1600, 45);
 turnPos(135);
 intake();
 motionController(950, 180);
 turnNeg(-92);
 motionController(900, 88);
 bruteForce(100, 100, 500);
 score(1);

 // Right bottom
  spitBlue();
  motionController(-600, 80);
  delay(1000);
  turnPos(65);
  intake();
  motionController(2200, 145);
  bruteForce(100, 100, 1000);
  score(1);

 // center
 spitBlue();
 motionController(-1500, 135);
 turnPos(135);
 intake();
 motionController(1100, 270);
 turnPos(95);
 motionController(300, 365);
 leftIntake.move(-127);
 bruteForce(127, 127, 1000);
 bruteForce(-127, -127, 500);
 bruteForce(80, 80, 1000);

 // Score
 score(2);
 bruteForce(-127, -127, 1000);
}

/**
* Runs the operator control code. This function will be started in its own task
* with the default priority and stack size whenever the robot is enabled via
* the Field Management System or the VEX Competition Switch in the operator
* control mode.
*
* If no competition control is connected, this function will run immediately
* following initialize().
*
* If the robot is disabled or communications is lost, the
* operator control task will be stopped. Re-enabling the robot will restart the
* task, not resume it from where it left off.
*/
void opcontrol() {
		bool a = false;
		bool b = false;

		while(true)
		{
      MEPrint(5, "Hehehe");
			if(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) > 10 &&
			master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) > 10)
			{
				leftFront.move(127);
				rightFront.move(50);
				leftBack.move(127);
				rightBack.move(50);
			}
			else if(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) > 10 &&
			master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) < -10)
			{
				leftFront.move(50);
				rightFront.move(127);
				leftBack.move(50);
				rightBack.move(127);
			}
			else if(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) < -10 &&
			master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) < -10)
			{
				leftFront.move(-127);
				rightFront.move(-50);
				leftBack.move(-127);
				rightBack.move(-50);
			}
			else if(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y) < -10 &&
			master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X) > 10)
			{
				leftFront.move(-50);
				rightFront.move(-127);
				leftBack.move(-50);
				rightBack.move(-127);
			}
			else if(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)>10)
			{
				leftFront.move(127);
				rightFront.move(127);
				leftBack.move(127);
				rightBack.move(127);
			}
			else if(master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)<-10)
			{
				leftFront.move(-127);
				rightFront.move(-127);
				leftBack.move(-127);
				rightBack.move(-127);
			}
			else if(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)>10)
			{
				leftFront.move(127);
				rightFront.move(-127);
				leftBack.move(127);
				rightBack.move(-127);
			}
			else if(master.get_analog(E_CONTROLLER_ANALOG_RIGHT_X)<-10)
			{
				leftFront.move(-127);
				rightFront.move(127);
				leftBack.move(-127);
				rightBack.move(127);
			}
			else if(master.get_analog(E_CONTROLLER_ANALOG_LEFT_X)<-10)
			{
				leftFront.move(-127);
				rightFront.move(127);
				leftBack.move(127);
				rightBack.move(-127);
			}
			else if(master.get_analog(E_CONTROLLER_ANALOG_LEFT_X)>10)
			{
				leftFront.move(127);
				rightFront.move(-127);
				leftBack.move(-127);
				rightBack.move(127);
			}
			else if(master.get_analog(E_CONTROLLER_ANALOG_LEFT_X)>10 && master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)>10)
			{
				leftFront.move(60);
				rightFront.move(-127);
				leftBack.move(-127);
				rightBack.move(60);
			}
			else if(master.get_analog(E_CONTROLLER_ANALOG_LEFT_X)>10 && master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)<-10)
			{
				leftFront.move(127);
				rightFront.move(-60);
				leftBack.move(-60);
				rightBack.move(127);
			}
			else if(master.get_analog(E_CONTROLLER_ANALOG_LEFT_X)<-10 && master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)>10)
			{
				leftFront.move(-127);
				rightFront.move(60);
				leftBack.move(60);
				rightBack.move(-127);
			}
			else if(master.get_analog(E_CONTROLLER_ANALOG_LEFT_X)<-10 && master.get_analog(E_CONTROLLER_ANALOG_LEFT_Y)<-10)
			{
				leftFront.move(-60);
				rightFront.move(127);
				leftBack.move(127);
				rightBack.move(-60);
			}
			else
			{
				leftFront.move(0);
				rightFront.move(0);
				leftBack.move(0);
				rightBack.move(0);
				leftFront.set_brake_mode(E_MOTOR_BRAKE_HOLD);
				rightFront.set_brake_mode(E_MOTOR_BRAKE_HOLD);
				leftBack.set_brake_mode(E_MOTOR_BRAKE_HOLD);
				rightBack.set_brake_mode(E_MOTOR_BRAKE_HOLD);
			}

			if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_R1))
			{
				if(b){
					b = false;
				}
				a = !a;
			}
			else if(master.get_digital_new_press(E_CONTROLLER_DIGITAL_R2))
			{
				if(a){
					a = false;
				}
				b = !b;
			}

			if(a)
			{
				leftIntake.move(127);
				rightIntake.move(127);
			}
			else if(b)
			{
				leftIntake.move(-127);
				rightIntake.move(-127);
			}
			else
			{
				leftIntake.move(0);
				rightIntake.move(0);
				leftIntake.set_brake_mode(E_MOTOR_BRAKE_HOLD);
				rightIntake.set_brake_mode(E_MOTOR_BRAKE_HOLD);
			}

			if(partner.get_digital(E_CONTROLLER_DIGITAL_L1))
			{
					frontRoller.move(100);
					backRoller.move(-127);
			}
			else if(partner.get_digital(E_CONTROLLER_DIGITAL_R1))
			{
				frontRoller.move(-127);
				backRoller.move(127);
			}
			else if(partner.get_digital(E_CONTROLLER_DIGITAL_L2))
			{
				frontRoller.move(100);
				backRoller.move(127);
			}
			else if(partner.get_digital(E_CONTROLLER_DIGITAL_R2))
			{
					frontRoller.move(90);
					backRoller.move(-90);
			}
			else
			{
				frontRoller.move(0);
				backRoller.move(0);
				frontRoller.set_brake_mode(E_MOTOR_BRAKE_HOLD);
				backRoller.set_brake_mode(E_MOTOR_BRAKE_HOLD);
			}
      if(partner.get_digital(E_CONTROLLER_DIGITAL_A)){
        Task spit_blue_task(spit_blue_task_fn);
      }
      delay(10);
		}
}
