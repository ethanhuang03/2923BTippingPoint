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
auto intake = Motor(7);
auto lift = Motor(8);

auto RightDrive = MotorGroup({frontRightDrive, backRightDrive, topRightDrive});
auto LeftDrive = MotorGroup({frontLeftDrive, backLeftDrive, topLeftDrive});

auto leftRotationSensor = RotationSensor(9);
auto rightRotationSensor = RotationSensor(10, true);
//auto interialSensor = IMU(11);

auto bumper0 = ADIButton('A');
auto bumper1 = ADIButton('B');
pros::ADIDigitalOut clamp_left('C');
pros::ADIDigitalOut clamp_right('D');
pros::ADIDigitalOut mogotilt_left('E');
pros::ADIDigitalOut mogotilt_right('F');
pros::ADIDigitalOut clampkiller('G');
pros::ADIDigitalOut piston2('H');

void initialize() {
	pros::lcd::initialize();
	selector::init();
	pros::lcd::set_text(0, "King's B | 2923B");
	drive = ChassisControllerBuilder()
		.withMotors(LeftDrive, RightDrive)
		//Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions({AbstractMotor::gearset::blue, (60.0 / 36.0)}, {{3.25_in, 13.7795_in}, imev5BlueTPR})
		.withSensors(
			leftRotationSensor,
			rightRotationSensor
		)
		// specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
    .withOdometry({{2.75_in, 7_in}, quadEncoderTPR})
		.withGains(
			{0.002, 0, 0.000197}, // Distance controller gains
			{0.00295, 0, 0.000090}, // Turn controller gains 0.00295
			{0.002, 0, 0.0001}  // Angle controller gains (helps drive straight)
		)
		.buildOdometry();
}

void disabled() {}

void competition_initialize() {}

void tank_drive(Controller controller) {
	pros::lcd::set_text(1, "Tank Drive");
	drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
}

void skills() {

}

void autonomous() {
	if(selector::auton == 1) { // Red Left

	}
	else if(selector::auton == 2) { // Red Left and Middle

	}
	else if(selector::auton == 3){ // Red Right

	}
	else if(selector::auton == 4) { // Red Right and Middle

	}
	else if(selector::auton == 5) { // Red Middle (From Left)

	}
	else if(selector::auton == 6) { // Red Middle (From Right)

	}
	if(selector::auton == -1) { // Blue Left

	}
	else if(selector::auton == -2) { // Blue Left and Middle

	}
	else if(selector::auton == -3){ // Blue Right

	}
	else if(selector::auton == -4) { // Blue Right and Middle

	}
	else if(selector::auton == -5) { // Blue Middle (From Left)

	}
	else if(selector::auton == -6) { // Blue Middle (From Right)

	}
	else if(selector::auton == 0){ //Skills
		skills();
	}
}

void opcontrol() {
	pros::lcd::set_text(2, "User Control");
	while(true){
		tank_drive(master);

		// Goal related stuff on the right hand
		// lift
		if(master.getDigital(ControllerDigital::R1) || partner.getDigital(ControllerDigital::R1)) {
			lift.moveVelocity(100);
		}
		else if(master.getDigital(ControllerDigital::R2) || partner.getDigital(ControllerDigital::R2)) {
			lift.moveVelocity(-100);
		}
		else {
			lift.moveVelocity(0);
		}

		// clamp
		if(master.getDigital(ControllerDigital::Y) || partner.getDigital(ControllerDigital::Y)) {
			clamp_left.set_value(true);
			clamp_right.set_value(true);
		}
		else if(master.getDigital(ControllerDigital::B) || partner.getDigital(ControllerDigital::B)) {
			clamp_left.set_value(false);
			clamp_right.set_value(false);
		}

		// MOGO stuff
		// intake
		if(master.getDigital(ControllerDigital::L1) || partner.getDigital(ControllerDigital::L1)) {
			intake.moveVelocity(100);
		}
		else if(master.getDigital(ControllerDigital::L2) || partner.getDigital(ControllerDigital::L2)) {
			intake.moveVelocity(-100);
		}
		else {
			intake.moveVelocity(0);
		}

		// mogo grab and tilt
		if(master.getDigital(ControllerDigital::right) || partner.getDigital(ControllerDigital::right)) {
			// pull in and tilt
			mogotilt_left.set_value(false);
			mogotilt_right.set_value(false);
		}
		else if(master.getDigital(ControllerDigital::down) || partner.getDigital(ControllerDigital::down)) {
			// push out and release
			mogotilt_left.set_value(true);
			mogotilt_right.set_value(true);
		}

		// the uncreachable buttons
		// clamp killer / top ring scorer
		if(master.getDigital(ControllerDigital::up) || partner.getDigital(ControllerDigital::up)) {
			clampkiller.set_value(true);
		}
		else if(master.getDigital(ControllerDigital::left) || partner.getDigital(ControllerDigital::left)) {
			clampkiller.set_value(false);
		}

		// some safety features?
		if(master.getDigital(ControllerDigital::X) || partner.getDigital(ControllerDigital::X)) {

		}
		else if(master.getDigital(ControllerDigital::A) || partner.getDigital(ControllerDigital::A)) {

		}
		else {

		}

		pros::delay(10);
	}
}
