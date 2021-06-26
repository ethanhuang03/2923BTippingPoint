#include "main.h"

okapi::Controller controller;

okapi::Motor frontRightDrive(1);
okapi::Motor backRightDrive(-2);
okapi::Motor frontLeftDrive(3);
okapi::Motor backLeftDrive(-4);

okapi::MotorGroup RightDrive({frontRightDrive, backRightDrive});
okapi::MotorGroup LeftDrive({frontLeftDrive, backLeftDrive});

okapi::IntegratedEncoder rightEncoder(backRightDrive);
okapi::IntegratedEncoder leftEncoder(backLeftDrive);

//Green gearset, 4 in wheel diam, 11.5 in wheel track
std::shared_ptr<ChassisController> drive = ChassisControllerBuilder().withMotors(LeftDrive, RightDrive).withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR}).withSensors(leftEncoder, rightEncoder).build();

okapi::ControllerButton R1(ControllerDigital::R1);
okapi::ControllerButton R2(ControllerDigital::R2);
okapi::ControllerButton L1(ControllerDigital::L1);
okapi::ControllerButton L2(ControllerDigital::L2);

okapi::ControllerButton X(ControllerDigital::X);
okapi::ControllerButton Y(ControllerDigital::Y);
okapi::ControllerButton A(ControllerDigital::A);
okapi::ControllerButton B(ControllerDigital::B);

okapi::ControllerButton Up(ControllerDigital::up);
okapi::ControllerButton Down(ControllerDigital::down);
okapi::ControllerButton Left(ControllerDigital::left);
okapi::ControllerButton Right(ControllerDigital::right);



void tank_drive(){
	drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
}

void arcade_drive(){
	drive->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::leftX));
}

void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

void initialize() {
	/*Logger::setDefaultLogger( //log output to pros terminal
		std::make_shared<Logger>(
			TimeUtilFactory::createDefault().getTimer(),
			"/ser/sout",
			Logger::LogLevel::debug
		)
	);*/
	pros::lcd::initialize();
	pros::lcd::set_text(1, "King's B | 2923B");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	drive->moveDistance(12_in);
	drive->turnAngle(90_deg);
}

void opcontrol() {
	tank_drive();
	//arcade_drive();

	pros::delay(10);
}
