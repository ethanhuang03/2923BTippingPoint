#include "main.h"
#include "ARMS/api.h"

Controller master(ControllerId::master);
Controller partner(ControllerId::partner);

std::shared_ptr<ChassisController> drive;

auto backRightDrive = Motor(-1);
auto frontRightDrive = Motor(-2);
auto topRightDrive = Motor(3);

auto backLeftDrive = Motor(4);
auto frontLeftDrive = Motor(5);
auto topLeftDrive = Motor(-6);

auto RightDrive = MotorGroup({frontRightDrive, backRightDrive, topRightDrive});
auto LeftDrive = MotorGroup({frontLeftDrive, backLeftDrive, topLeftDrive});

void initialize() {
	pros::lcd::initialize();
	selector::init();
	pros::lcd::set_text(0, "King's B | 2923B");
	drive = ChassisControllerBuilder()
		.withMotors(LeftDrive, RightDrive)
		//Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions({AbstractMotor::gearset::blue, (60.0 / 36.0)}, {{3.25_in, 13.7795_in}, imev5BlueTPR})
		.withOdometry() // or .withOdometry({{4_in, 15_in}, quadEncoderTPR})
		.withGains(
			{0.002, 0, 0.000197}, // Distance controller gains
			{0.00295, 0, 0.000090}, // Turn controller gains 0.00295
			{0.002, 0, 0.0001}  // Angle controller gains (helps drive straight)
		)
		.buildOdometry();
}

void disabled() {}

void competition_initialize() {}

void tank_drive(){
	pros::lcd::set_text(1, "Tank Drive");
	drive->getModel()->tank(master.getAnalog(ControllerAnalog::leftY), master.getAnalog(ControllerAnalog::rightY));
}

void red_front(){
}

void red_back(){
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

		pros::delay(10);
	}
}
