
#include "main.h"

Controller controller;

//Green gearset, 4 in wheel diam, 11.5 in wheel track
std::shared_ptr<ChassisController> drive;

auto frontRightDrive = Motor(1);
auto backRightDrive = Motor(2);
auto frontLeftDrive = Motor(-3);
auto backLeftDrive = Motor(-4);

auto RightDrive = MotorGroup({frontRightDrive, backRightDrive});
auto LeftDrive = MotorGroup({frontLeftDrive, backLeftDrive});

auto rightEncoder = IntegratedEncoder(backRightDrive);
auto leftEncoder = IntegratedEncoder(backLeftDrive);

auto leftADI_Encoder = ADIEncoder('A', 'B'); // Left encoder in ADI ports A & B
auto rightADI_Encoder = ADIEncoder('C', 'D', true);  // Right encoder in ADI ports C & D (reversed)

auto button_R1 = ControllerButton(ControllerDigital::R1);
auto button_R2 = ControllerButton(ControllerDigital::R2);
auto button_L1 = ControllerButton(ControllerDigital::L1);
auto button_L2 = ControllerButton(ControllerDigital::L2);

auto button_X = ControllerButton(ControllerDigital::X);
auto button_Y = ControllerButton(ControllerDigital::Y);
auto button_A = ControllerButton(ControllerDigital::A);
auto button_B = ControllerButton(ControllerDigital::B);

auto button_Up = ControllerButton(ControllerDigital::up);
auto button_Down = ControllerButton(ControllerDigital::down);
auto button_Left = ControllerButton(ControllerDigital::left);
auto button_Right = ControllerButton(ControllerDigital::right);

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
	drive = ChassisControllerBuilder()
		.withMotors(LeftDrive, RightDrive)
		.withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
		.withSensors(leftEncoder, rightEncoder)
		.withOdometry()
		.buildOdometry();
/*
	drive = ChassisControllerBuilder()
		.withMotors(LeftDrive, RightDrive)
		.withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
		.withSensors(leftADI_Encoder, rightADI_Encoder)
		// Specify the tracking wheels diam (2.75 in), track (7 in), and TPR (360)
		.withOdometry({{2.75_in, 7_in}, quadEncoderTPR})
		.buildOdometry();
*/
}

void disabled() {}

void competition_initialize() {}

void tank_drive(){
	drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
}

void arcade_drive(){
	drive->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::leftX));
}

void autonomous() {
	drive->moveDistance(12_in);
	drive->turnAngle(90_deg);
}

void opcontrol() {
	tank_drive();
	//arcade_drive();

	pros::delay(10);
}
