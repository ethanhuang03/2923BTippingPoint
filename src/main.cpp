#include "main.h"
#include "autoSelect/selection.h"

Controller master(ControllerId::master);
Controller partner(ControllerId::partner);

std::shared_ptr<OdomChassisController> drive;
std::shared_ptr<AsyncMotionProfileController> driveController;
std::shared_ptr<AsyncPositionController<double, double>> asyncLift;

Motor backRightDrive(4);
Motor frontRightDrive(5);
Motor topRightDrive(6);
Motor backLeftDrive(-1);
Motor frontLeftDrive(-2);
Motor topLeftDrive(-3);
Motor intake(-8);
Motor lift(-7);

MotorGroup RightDrive({frontRightDrive, backRightDrive, topRightDrive});
MotorGroup LeftDrive({frontLeftDrive, backLeftDrive, topLeftDrive});

RotationSensor leftRotationSensor(13);
RotationSensor rightRotationSensor(12, true);
RotationSensor centerRotationSensor(11);
//IMU interialSensor(11);

ADIButton frontBumper('F');
ADIButton backBumper('G');

pros::ADIDigitalOut tilt('B');
pros::ADIDigitalOut backClamp('A');
pros::ADIDigitalOut frontClamp('C');
pros::ADIDigitalOut flap('D');
pros::ADIDigitalOut wings('E');


void piston(pros::ADIDigitalOut piston, bool intially_extended, bool extend) {
	if(intially_extended) {
		if(extend) {
			piston.set_value(false);
		}
		else {
			piston.set_value(true);
		}
	}
	else {
		if(extend) {
			piston.set_value(true);
		}
		else {
			piston.set_value(false);
		}
	}
}


void initialize() {
	pros::lcd::initialize();
	selector::init();
	pros::lcd::set_text(0, "King's B | 2923B");
	piston(backClamp, true, false);
	drive = ChassisControllerBuilder()
		.withLogger(
			std::make_shared<Logger>(
				TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
				"/ser/sout", // Output to the PROS terminal
				Logger::LogLevel::debug // Most verbose log level
			)
		)
		.withMotors(LeftDrive, RightDrive)
		.withDimensions({AbstractMotor::gearset::blue, (60.0 / 36.0)}, {{3.25_in, 14.8333_in}, imev5BlueTPR})
		.withSensors(
			leftRotationSensor,
			rightRotationSensor,
			centerRotationSensor
		)
    	.withOdometry({{2.75_in, 8.5_in, 3.5_in, 2.75_in}, quadEncoderTPR})
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

	driveController = AsyncMotionProfileControllerBuilder()
		.withLimits({
			1.0, // Maximum linear velocity of the Chassis in m/s
			2.0, // Maximum linear acceleration of the Chassis in m/s/s
			10.0 // Maximum linear jerk of the Chassis in m/s/s/s
		})
		.withOutput(drive)
		.buildMotionProfileController();

	asyncLift = AsyncPosControllerBuilder()
    	.withMotor(8) // lift motor port 8
    	.build();
}


void disabled() {}


void competition_initialize() {}


void tank_drive(Controller controller) {
	drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
}


void skills() {}


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
    	asyncLift->setTarget(50); // raise the lift
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
    	asyncLift->setTarget(50); // raise the lift
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
    	asyncLift->setTarget(50); // raise the lift
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
	bool backClampToggle = false;
	bool frontClampToggle = false;
	bool flapToggle = false;
	bool swiperToggle = false;
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
		
		// MOGO stuff
		// intake
		if(master.getDigital(ControllerDigital::L1) || partner.getDigital(ControllerDigital::L1)) {
			intake.moveVelocity(600);
		}
		else if(master.getDigital(ControllerDigital::L2) || partner.getDigital(ControllerDigital::L2)) {
			intake.moveVelocity(-600);
		}
		else {
			intake.moveVelocity(0);
		}

		// clamp
		if(master.getDigital(ControllerDigital::Y) || partner.getDigital(ControllerDigital::Y)) {
			if (frontClampToggle) {
				frontClampToggle = false;
				piston(frontClamp, true, true);
				pros::delay(200);
			}
			else {
				frontClampToggle = true;
				piston(frontClamp, true, false);
				pros::delay(200);
			}
		}
		
		// mogo grab and tilt
		if(master.getDigital(ControllerDigital::right) || partner.getDigital(ControllerDigital::right)) {
			if (backClampToggle) {
				backClampToggle = false;
				piston(tilt, true, true);
				pros::delay(250);
				piston(backClamp, true, false);
			}
			else {
				backClampToggle = true;
				piston(backClamp, true, true);
				pros::delay(250);
				piston(tilt, true, false);
			}
			
		}

		// the uncreachable buttons
		// clamp killer / top ring scorer
		if(master.getDigital(ControllerDigital::up) || partner.getDigital(ControllerDigital::up)) {
			piston(flap, false, true);
		}
		else if(master.getDigital(ControllerDigital::left) || partner.getDigital(ControllerDigital::left)) {
			piston(flap, false, false);
		}

		// some safety features?
		if(master.getDigital(ControllerDigital::X) || partner.getDigital(ControllerDigital::X)) {
			piston(wings, false, true);
		}
		else if(master.getDigital(ControllerDigital::A) || partner.getDigital(ControllerDigital::A)) {
			piston(wings, false, false);
		}
		
		pros::delay(10);
	}
}