#include "main.h"
#include "autoSelect/selection.h"
#include "pros-grafana-lib/api.h"

Controller master(ControllerId::master);
Controller partner(ControllerId::partner);

std::shared_ptr<OdomChassisController> drive;
std::shared_ptr<AsyncMotionProfileController> driveController;
std::shared_ptr<AsyncPositionController<double, double>> asyncLift;

Motor backRightDrive(1);
Motor frontRightDrive(2);
Motor topRightDrive(3);
Motor backLeftDrive(-4);
Motor frontLeftDrive(-5);
Motor topLeftDrive(-6);
Motor intake(7);
Motor lift(8);

MotorGroup RightDrive({frontRightDrive, backRightDrive, topRightDrive});
MotorGroup LeftDrive({frontLeftDrive, backLeftDrive, topLeftDrive});

RotationSensor leftRotationSensor(9);
RotationSensor rightRotationSensor(10, true);
RotationSensor centerRotationSensor(11);
//IMU interialSensor(11);

ADIButton frontBumper('A');
ADIButton backBumper('B');

pros::ADIDigitalOut frontClamp('C');
pros::ADIDigitalOut backClamp('D');
pros::ADIDigitalOut tilt('E');
pros::ADIDigitalOut flap('F');
pros::ADIDigitalOut wings('G');

auto manager = std::make_shared<grafanalib::GUIManager>();

void initialize() {
	pros::lcd::initialize();
	selector::init();
	pros::lcd::set_text(0, "King's B | 2923B");
	drive = ChassisControllerBuilder()
		.withLogger(
			std::make_shared<Logger>(
				TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
				"/ser/sout", // Output to the PROS terminal
				Logger::LogLevel::debug // Most verbose log level
			)
		)
		.withMotors(LeftDrive, RightDrive)
		//Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions({AbstractMotor::gearset::blue, (60.0 / 36.0)}, {{3.25_in, 13.7795_in}, imev5BlueTPR})
		.withSensors(
			leftRotationSensor,
			rightRotationSensor
			// centerRotationSensor
		)
		// specify the tracking wheels diameter (2.75 in), track (7 in), and TPR (360)
		// specify the middle encoder distance (1 in) and diameter (2.75 in)
    	.withOdometry({{2.75_in, 7_in, 1_in, 2.75_in}, quadEncoderTPR})
		.withGains(
			{0.002, 0, 0.000197}, // Distance controller gains
			{0.00295, 0, 0.000090}, // Turn controller gains 0.00295
			{0.002, 0, 0.0001}  // Angle controller gains (helps drive straight)
		)
		// Stuff Below Here is Experimental
		.withDerivativeFilters(
			std::make_unique<AverageFilter<3>>(), // Distance controller filter
			std::make_unique<AverageFilter<3>>(), // Turn controller filter
			std::make_unique<AverageFilter<3>>()  // Angle controller filter
		)
		.withClosedLoopControllerTimeUtil(50, 5, 250_ms) // The minimum error to be considered settled, error derivative to be considered settled, time within atTargetError to be considered settled
		.buildOdometry();

	std::shared_ptr<AsyncMotionProfileController> driveController =
		AsyncMotionProfileControllerBuilder()
		/*
		.withLimits({
			1.0, // Maximum linear velocity of the Chassis in m/s
			2.0, // Maximum linear acceleration of the Chassis in m/s/s
			10.0 // Maximum linear jerk of the Chassis in m/s/s/s
		})
		*/
		.withOutput(drive)
		.buildMotionProfileController();

	asyncLift = AsyncPosControllerBuilder()
    .withMotor(8) // lift motor port 8
    .withGains({0.001, 0.0001, 0.0001})
    .build();

	manager->setRefreshRate(101); // > 100 if wireless

	grafanalib::Variable<MotorGroup> RightDriveVar("RightDrive", RightDrive);
	grafanalib::Variable<MotorGroup> LeftDriveVar("LeftDrive", LeftDrive);

	grafanalib::Variable<Motor> intakeVar("intake", intake);
	grafanalib::Variable<Motor> liftVar("lift", lift);

	grafanalib::Variable<RotationSensor> leftRotationSensorVar("leftRotationSensor", leftRotationSensor);
	grafanalib::Variable<RotationSensor> rightRotationSensorVar("rightRotationSensor", rightRotationSensor);
	grafanalib::Variable<RotationSensor> centerRotationSensorVar("centerRotationSensor", centerRotationSensor);

	grafanalib::Variable<ADIButton> frontBumperVar("frontBumper", frontBumper);
	grafanalib::Variable<ADIButton> backBumperVar("backBumper", backBumper);

	RightDriveVar.add_getter("Temperature", &MotorGroup::getTemperature);
	RightDriveVar.add_getter("Actual Velocity", &MotorGroup::getActualVelocity);
	RightDriveVar.add_getter("Target Velocity", &MotorGroup::getTargetVelocity);
	RightDriveVar.add_getter("Voltage", &MotorGroup::getVoltage);
	RightDriveVar.add_getter("Efficiency", &MotorGroup::getEfficiency);
	RightDriveVar.add_getter("Current", &MotorGroup::getCurrentDraw);
	RightDriveVar.add_getter("Power", &MotorGroup::getPower);
	RightDriveVar.add_getter("Torque", &MotorGroup::getTorque);
	RightDriveVar.add_getter("Position", &MotorGroup::getPosition);
	RightDriveVar.add_getter("Target Position", &MotorGroup::getTargetPosition);

	LeftDriveVar.add_getter("Temperature", &MotorGroup::getTemperature);
	LeftDriveVar.add_getter("Actual Velocity", &MotorGroup::getActualVelocity);
	LeftDriveVar.add_getter("Target Velocity", &MotorGroup::getTargetVelocity);
	LeftDriveVar.add_getter("Voltage", &MotorGroup::getVoltage);
	LeftDriveVar.add_getter("Efficiency", &MotorGroup::getEfficiency);
	LeftDriveVar.add_getter("Current", &MotorGroup::getCurrentDraw);
	LeftDriveVar.add_getter("Power", &MotorGroup::getPower);
	LeftDriveVar.add_getter("Torque", &MotorGroup::getTorque);
	LeftDriveVar.add_getter("Position", &MotorGroup::getPosition);
	LeftDriveVar.add_getter("Target Position", &MotorGroup::getTargetPosition);

	intakeVar.add_getter("Temperature", &Motor::getTemperature);
	intakeVar.add_getter("Actual Velocity", &Motor::getActualVelocity);
	intakeVar.add_getter("Target Velocity", &Motor::getTargetVelocity);
	intakeVar.add_getter("Voltage", &Motor::getVoltage);
	intakeVar.add_getter("Efficiency", &Motor::getEfficiency);
	intakeVar.add_getter("Current", &Motor::getCurrentDraw);
	intakeVar.add_getter("Power", &Motor::getPower);
	intakeVar.add_getter("Torque", &Motor::getTorque);
	intakeVar.add_getter("Position", &Motor::getPosition);
	intakeVar.add_getter("Target Position", &Motor::getTargetPosition);

	liftVar.add_getter("Temperature", &Motor::getTemperature);
	liftVar.add_getter("Actual Velocity", &Motor::getActualVelocity);
	liftVar.add_getter("Target Velocity", &Motor::getTargetVelocity);
	liftVar.add_getter("Voltage", &Motor::getVoltage);
	liftVar.add_getter("Efficiency", &Motor::getEfficiency);
	liftVar.add_getter("Current", &Motor::getCurrentDraw);
	liftVar.add_getter("Power", &Motor::getPower);
	liftVar.add_getter("Torque", &Motor::getTorque);
	liftVar.add_getter("Position", &Motor::getPosition);
	liftVar.add_getter("Target Position", &Motor::getTargetPosition);

	leftRotationSensorVar.add_getter("Rotation", &RotationSensor::get);
	rightRotationSensorVar.add_getter("Rotation", &RotationSensor::get);
	centerRotationSensorVar.add_getter("Rotation", &RotationSensor::get);

	frontBumperVar.add_getter("Pressed", &ADIButton::isPressed);
	backBumperVar.add_getter("Pressed", &ADIButton::isPressed);

	manager->registerDataHandler(&RightDriveVar);
	manager->registerDataHandler(&LeftDriveVar);
	manager->registerDataHandler(&intakeVar);
	manager->registerDataHandler(&liftVar);
	manager->registerDataHandler(&leftRotationSensorVar);
	manager->registerDataHandler(&rightRotationSensorVar);
	manager->registerDataHandler(&centerRotationSensorVar);
	manager->registerDataHandler(&frontBumperVar);
	manager->registerDataHandler(&backBumperVar);

	manager->startTask();

}

void disabled() {}

void competition_initialize() {}

void tank_drive(Controller controller) {
	pros::lcd::set_text(1, "Tank Drive");
	drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
}

void skills() {

}

void left() {
	drive->driveToPoint({3.7_ft, 0.7_ft}); // goal around half a foot
	if (frontBumper.isPressed()) {
		frontClamp.set_value(true); // clamp the goal
		pros::delay(200);
		drive->driveToPoint({-1_ft, 0_ft}, true); // has goal, continue to drive back
	}
	drive->driveToPoint({1_ft, 3.5_ft}, true); // good position
}

void left_middle() {
	drive->driveToPoint({3.7_ft, 0.7_ft}); // goal around half a foot
	if (frontBumper.isPressed()) {
		frontClamp.set_value(true); // clamp the goal
		pros::delay(200);
		drive->driveToPoint({-1_ft, 0_ft}, true); // has goal, continue to drive back
	}
	else { // go for middle goal
		drive->driveToPoint({3.7_ft, 3.3_ft}); // drive towards goal
		if (frontBumper.isPressed()) { // conserve air
			pros::delay(200);
			frontClamp.set_value(true); // clamp the goal
			pros::delay(200);
		}
		drive->driveToPoint({1_ft, 3.5_ft}, true); // good position
	}
}

void right() {
	drive->driveToPoint({3.7_ft, 0_ft}); // goal around half a foot
	if (frontBumper.isPressed()) {
		frontClamp.set_value(true); // clamp the goal
		pros::delay(200);
		drive->driveToPoint({-1_ft, 0_ft}, true); // has goal, continue to drive back
	}
	else { // go for middle goal
		drive->driveToPoint({1.6_ft, 0_ft}, true);
		drive->turnToAngle(-80_deg); // face the alliance goal
		drive->moveDistance(-2.2_ft); // drive into alliance goal
		backClamp.set_value(false); // clamp down
		pros::delay(200);
		tilt.set_value(false);
		drive->driveToPoint({2.2_ft, -2.2_ft});
    asyncLift->setTarget(50_deg); // raise the lift
		drive->turnToAngle(-90_deg); // positioned at rings
		intake.moveVelocity(100); // intake rings
		drive->driveToPoint({2.2_ft, -6_ft});
		drive->driveToPoint({2.2_ft, -3_ft}, true);
		drive->turnToAngle(0_deg); // face forward
	}
}

void right_middle() {
	drive->driveToPoint({3.7_ft, 0_ft}); // goal around half a foot
	if (frontBumper.isPressed()) {
		frontClamp.set_value(true); // clamp the goal
		pros::delay(200);
		drive->driveToPoint({-1_ft, 0_ft}, true); // has goal, continue to drive back
	}
	else { // go for middle goal
		drive->driveToPoint({2.9_ft, 0_ft}, true);
		drive->driveToPoint({3.8_ft, -3_ft}); // drive towards goal
		pros::delay(200);
		frontClamp.set_value(true); // clamp the goal
		pros::delay(200);
		drive->driveToPoint({1.4_ft, 0_ft}, true);
		drive->turnToAngle(-80_deg); // face the alliance goal
		drive->moveDistance(-2.2_ft); //drive into alliance goal
		backClamp.set_value(false); // clamp down
		pros::delay(200);
		tilt.set_value(false);
		drive->driveToPoint({2.2_ft, -2.2_ft});
    asyncLift->setTarget(50_deg); // raise the lift
		drive->turnToAngle(-90_deg); // positioned at rings
		intake.moveVelocity(100); // intake rings
		drive->driveToPoint({2.2_ft, -6_ft});
		drive->driveToPoint({2.2_ft, -3_ft}, true);
		drive->turnToAngle(0_deg); // face forward
	}
}

void middle_left() {
	drive->driveToPoint({4_ft, 3.8_ft}); // goal around half a foot
	if (frontBumper.isPressed()) {
		frontClamp.set_value(true); // clamp the goal
		pros::delay(200);
		drive->driveToPoint({0_ft, 0_ft}, true); // has goal, continue to drive back
	}
	else { // good position
		drive->driveToPoint({2.2_ft, -6_ft});
		drive->driveToPoint({2.2_ft, -3_ft}, true);
		drive->turnToAngle(0_deg); // face forward
	}
}

void middle_right() {
	drive->driveToPoint({4_ft, -2.4_ft}); // goal around half a foot
	if (frontBumper.isPressed()) {
		frontClamp.set_value(true); // clamp the goal
		pros::delay(200);
		drive->driveToPoint({0_ft, 0_ft}, true); // has goal, continue to drive back
	}
	else { // good position
		drive->driveToPoint({2.2_ft, -3_ft}, true);
		drive->turnToAngle(0_deg); // face forward
	}
}

void wings_left() {
	drive->driveToPoint({2.7_ft, 0.7_ft}); // goal around half a foot
	drive->turnToAngle(40_deg); // swat goal
	drive->moveDistance(1.8_ft); // move forward towards goal
	if (frontBumper.isPressed()) {
		frontClamp.set_value(true); // clamp the goal
		pros::delay(200);
		drive->driveToPoint({-1_ft, 0_ft}, true); // has goal, continue to drive back
	}
	else { // go for middle goal
		drive->driveToPoint({1_ft, 3.5_ft}, true); // good position
	}
}

void wings_right() {
	drive->driveToPoint({2.7_ft, 0_ft}); // goal around half a foot
	drive->turnToAngle(40_deg); // swat goal
	drive->moveDistance(1.8_ft); // move forward towards goal
	if (frontBumper.isPressed()) {
		frontClamp.set_value(true); // clamp the goal
		pros::delay(200);
		drive->driveToPoint({-1_ft, 0_ft}, true); // has goal, continue to drive back
	}
	else { // go for alliance goal
		drive->driveToPoint({1.6_ft, 0_ft}, true);
		drive->turnToAngle(-80_deg); // face the alliance goal
		drive->moveDistance(-2.2_ft); //drive into alliance goal
		backClamp.set_value(false); // clamp down
		pros::delay(200);
		tilt.set_value(false); // tilt
		drive->driveToPoint({2.2_ft, -2.2_ft});
    asyncLift->setTarget(50_deg); // raise the lift
		drive->turnToAngle(-90_deg); // positioned at rings
		intake.moveVelocity(100); // intake rings
		drive->driveToPoint({2.2_ft, -6_ft});
		drive->driveToPoint({2.2_ft, -3_ft}, true);
		drive->turnToAngle(0_deg); // face forward
	}
}

void autonomous() {
	drive->setState({0_in, 0_in, 0_deg});
	if(selector::auton == 1) { // Red Left
		left();
	}
	else if(selector::auton == 2) { // Red Left and Middle
		left_middle();
	}
	else if(selector::auton == 3){ // Red Right
		right();
	}
	else if(selector::auton == 4) { // Red Right and Middle
		right_middle();
	}
	else if(selector::auton == 5) { // Red Middle (From Left)
		middle_left();
	}
	else if(selector::auton == 6) { // Red Middle (From Right)
		middle_right();
	}
	else if(selector::auton == 7) { // Wings (Left)
		wings_left();
	}
	else if(selector::auton == 8) { // Wings (Right)
		wings_right();
	}
	else if(selector::auton == -1) { // Blue Left
		left();
	}
	else if(selector::auton == -2) { // Blue Left and Middle
		left_middle();
	}
	else if(selector::auton == -3){ // Blue Right
		right();
	}
	else if(selector::auton == -4) { // Blue Right and Middle
		right_middle();
	}
	else if(selector::auton == -5) { // Blue Middle (From Left)
		middle_left();
	}
	else if(selector::auton == -6) { // Blue Middle (From Right)
		middle_right();
	}
	else if(selector::auton == -7) { // Wings (Left)
		wings_left();
	}
	else if(selector::auton == -8) { // Wings (Right)
		wings_right();
	}
	else if(selector::auton == 0){ //Skills
		skills();
	}
}

void opcontrol() {
	pros::lcd::set_text(2, "User Control");
	while(true){
		tank_drive(master);

		// Front Goal related stuff on the right hand
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
			frontClamp.set_value(true);
		}
		else if(master.getDigital(ControllerDigital::B) || partner.getDigital(ControllerDigital::B)) {
			frontClamp.set_value(false);
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
			backClamp.set_value(true);
			pros::delay(500);
			tilt.set_value(false);
		}
		else if(master.getDigital(ControllerDigital::down) || partner.getDigital(ControllerDigital::down)) {
			// push out and release
			tilt.set_value(true);
			pros::delay(500);
			backClamp.set_value(false);
		}

		// the uncreachable buttons
		// clamp killer / top ring scorer
		if(master.getDigital(ControllerDigital::up) || partner.getDigital(ControllerDigital::up)) {
			flap.set_value(true);
		}
		else if(master.getDigital(ControllerDigital::left) || partner.getDigital(ControllerDigital::left)) {
			flap.set_value(false);
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
