#include "main.h"
#include "ARMS/api.h"

Controller master(ControllerId::master);
Controller partner(ControllerId::partner);

std::shared_ptr<ChassisController> drive;

auto backRightDrive = Motor(-1);
auto frontRightDrive = Motor(-2);
auto backLeftDrive = Motor(3);
auto frontLeftDrive = Motor(4);

auto lift = Motor(5);
auto intake = Motor(6);
auto mogo = Motor(7);
auto clamp = Motor(8);

auto RightDrive = MotorGroup({frontRightDrive, backRightDrive});
auto LeftDrive = MotorGroup({frontLeftDrive, backLeftDrive});

bool spin_flag = false;
int spin_velocity = 100;
int spin_v = 100;

void move(float ft){

}

/*
void turn(int degrees, bool right){
	if(right){
		backRightDrive.moveAbsolute(-degrees);
		frontRightDrive.moveAbsolute(-degrees)
		backLeftDrive.moveAbsolute(degrees);
		backLeftDrive.moveAbsolute(degrees);
	}
	else{
		backRightDrive.moveAbsolute(degrees);
		frontRightDrive.moveAbsolute(degrees)
		backLeftDrive.moveAbsolute(-degrees);
		backLeftDrive.moveAbsolute(-degrees);
	}
}
*/

void initialize() {
	pros::lcd::initialize();
	selector::init();
	pros::lcd::set_text(0, "King's B | 2923B");
	drive = ChassisControllerBuilder()
		.withMotors(LeftDrive, RightDrive)
		//Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions({AbstractMotor::gearset::green, (36.0 / 60.0)}, {{3.25_in, 13.7795_in}, imev5GreenTPR})
		.withOdometry() // or .withOdometry({{4_in, 15_in}, quadEncoderTPR})
		.withGains(
			{0.002, 0, 0.000197}, // Distance controller gains
			{0.00295, 0, 0.000090}, // Turn controller gains 0.00295
			{0.002, 0, 0.0001}  // Angle controller gains (helps drive straight)
		)
		.buildOdometry();
		lift.setBrakeMode(AbstractMotor::brakeMode::hold);
}

std::shared_ptr<AsyncPositionController<double, double>> mogoController =
  AsyncPosControllerBuilder()
    .withMotor(7)
    .build();

std::shared_ptr<AsyncPositionController<double, double>> liftController =
  AsyncPosControllerBuilder()
    .withMotor(5)
    .build();

std::shared_ptr<AsyncPositionController<double, double>> intakeController =
  AsyncPosControllerBuilder()
    .withMotor(6)
    .build();

std::shared_ptr<AsyncPositionController<double, double>> clampController =
  AsyncPosControllerBuilder()
    .withMotor(8)
    .build();

void disabled() {}

void competition_initialize() {}

void tank_drive(){
	pros::lcd::set_text(1, "Tank Drive");
	drive->getModel()->tank(master.getAnalog(ControllerAnalog::rightY), master.getAnalog(ControllerAnalog::leftY));
}

void red_front(){
	int old = mogoController->getTarget();
	mogoController->setTarget(1300);
	liftController->setTarget(500);
	mogoController->waitUntilSettled();

	drive->moveDistanceAsync(-2_ft);
	drive->waitUntilSettled();

	mogoController->setTarget(old);
	mogoController->waitUntilSettled();

	intakeController->setMaxVelocity(100);
	intakeController->setTarget(-1000000);

	drive->setMaxVelocity(50);
	drive->moveDistanceAsync(2.8_ft);
/*
	drive->turnAngle(-95_deg);
	drive->moveDistanceAsync(1_ft);
	drive->waitUntilSettled();
	drive->turnAngle(-95_deg);
	drive->setMaxVelocity(50);
	drive->moveDistanceAsync(2.5_ft);
	drive->waitUntilSettled();

	drive->moveDistanceAsync(-5_ft);
	drive->waitUntilSettled();
*/
}

void red_back(){
	int old = mogoController->getTarget();
	mogoController->setTarget(1300);
	liftController->setTarget(500);
	mogoController->waitUntilSettled();

	drive->moveDistanceAsync(-2_ft);
	drive->waitUntilSettled();

	mogoController->setTarget(old);
	mogoController->waitUntilSettled();

	intakeController->setMaxVelocity(100);
	intakeController->setTarget(-1000000);

	drive->setMaxVelocity(50);
	drive->moveDistanceAsync(2.8_ft);
}

void do_nothing(){

}

void blue_front(){
	red_front();
}

void blue_back(){
	red_back();
}

void skills(){

}

void autonomous() {
	if(selector::auton == 1){ //Red Front
		red_front();
	}
	else if(selector::auton == 2){ //Red Back
		red_back();
	}
	else if(selector::auton == 3){ //Do Nothing
		do_nothing();
	}
	else if(selector::auton == -1){ //Blue Front
		blue_front();
	}
	else if(selector::auton == -2){ //Blue Back
		blue_back();
	}
	else if(selector::auton == -3){ //Do Nothing
		do_nothing();
	}
	else if(selector::auton == 0){ //Skills
		skills();
	}
}

void opcontrol() {
	pros::lcd::set_text(2, "User Control");
	while(true){
		tank_drive();

		if(partner.getDigital(ControllerDigital::R1) || master.getDigital(ControllerDigital::R1)){
			spin_velocity = -200;
			intake.moveVelocity(spin_velocity);
		}
		else if(partner.getDigital(ControllerDigital::R2) || master.getDigital(ControllerDigital::R2)){
			spin_velocity = 200;
			intake.moveVelocity(spin_velocity);
		}

		if(partner.getDigital(ControllerDigital::L1) || master.getDigital(ControllerDigital::L1)){
			mogo.moveVelocity(-200);
		}
		else if(partner.getDigital(ControllerDigital::L2) || master.getDigital(ControllerDigital::L2)){
			mogo.moveVelocity(200);
		}
		else{
			mogo.moveVelocity(0);
		}

		if(partner.getDigital(ControllerDigital::X) || master.getDigital(ControllerDigital::X)){
			clamp.moveVelocity(-100);
		}
		else if(partner.getDigital(ControllerDigital::Y) || master.getDigital(ControllerDigital::Y)){
			clamp.moveVelocity(100);
		}
		else{
			clamp.moveVelocity(0);
		}

		if(partner.getDigital(ControllerDigital::up) || master.getDigital(ControllerDigital::up)){
			lift.moveVelocity(100);
		}
		else if(partner.getDigital(ControllerDigital::down) || master.getDigital(ControllerDigital::down)){
			lift.moveVelocity(-100);
		}
		else{
			lift.moveVelocity(0);
		}

		pros::delay(10);
	}
}
