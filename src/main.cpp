#include "main.h"
#include "autoSelect/selection.h"

Controller master(ControllerId::master);
Controller partner(ControllerId::partner);
Controller driver = master;

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

ADIButton frontBumper('H');

pros::ADIDigitalOut backClamp('A');
pros::ADIDigitalOut tilt('B');
pros::ADIDigitalOut frontClamp('D');
pros::ADIDigitalOut flap('C');
pros::ADIDigitalOut swiper('E');

bool intake_toggle = false;
int intakeDirection = 0;


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


void intake_switcher(bool toggle) {
	if(toggle) {
		if(driver.getDigital(ControllerDigital::L1) || partner.getDigital(ControllerDigital::L1)) {
			if (intakeDirection == 1) {
				intakeDirection = 0;
				intake.moveVelocity(0);
				pros::delay(250);
			}
			else {
				intakeDirection = 1;
				intake.moveVelocity(600);
				pros::delay(250);
			}
		}
		else if(driver.getDigital(ControllerDigital::L2) || partner.getDigital(ControllerDigital::L2)) {
			if (intakeDirection == -1) {
				intakeDirection = 0;
				intake.moveVelocity(0);
				pros::delay(250);
			}
			else {
				intakeDirection = -1;
				intake.moveVelocity(-600);
				pros::delay(250);
			}
		}
	}
	else{
		if(driver.getDigital(ControllerDigital::L1) || partner.getDigital(ControllerDigital::L1)) {
			intake.moveVelocity(600);
		}
		else if(driver.getDigital(ControllerDigital::L2) || partner.getDigital(ControllerDigital::L2)) {
			intake.moveVelocity(-600);
		}
		else {
			intake.moveVelocity(0);
		}
	}
}


void initialize() {
	selector::init();
	
	drive = ChassisControllerBuilder()
		.withLogger(
			std::make_shared<Logger>(
				TimeUtilFactory::createDefault().getTimer(), // It needs a Timer
				"/ser/sout", // Output to the PROS terminal
				Logger::LogLevel::debug // Most verbose log level
			)
		)
		.withMotors(LeftDrive, RightDrive)
		.withDimensions({AbstractMotor::gearset::blue, (60.0 / 36.0)}, {{3.25_in, 37.8_cm}, imev5BlueTPR})
		.withSensors(
			leftRotationSensor,
			rightRotationSensor,
			centerRotationSensor
		)
    	.withOdometry({{2.75_in, 8.5_in, 3.5_in, 2.75_in}, quadEncoderTPR}, StateMode::CARTESIAN) //2.86 8.5
		.withGains(
			{0, 0, 0}, // Distance controller gains
			{0, 0, 0}, // Turn controller gains 0.00295, p=0.00275
			{0, 0, 0}  // Angle controller gains (helps drive straight)
		)
		// Stuff Below Here is Experimental
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

	lift.setBrakeMode(AbstractMotor::brakeMode::hold);

	while(true) {
		if(master.getDigital(ControllerDigital::left) || partner.getDigital(ControllerDigital::left)) {
			drive->setState({0_ft, 0_ft, 0_deg}); //needs to be offset
			master.setText(0, 0, "0ft, 0ft, 0deg");
			partner.setText(0, 0, "0ft, 0ft, 0deg");
			break;
		}
		else if(master.getDigital(ControllerDigital::right) || partner.getDigital(ControllerDigital::right)) {
			drive->setState({12_ft, 0_ft, 0_deg});
			master.setText(0, 0, "12ft, 0ft, 0deg");
			partner.setText(0, 0, "12ft, 0ft, 0deg");
			break; //needs to be offset
		}
	}
	master.clear();
	partner.clear();
	while(true) {
		master.clearLine(1);
		partner.clearLine(1);
		auto state = drive->getState();
		if(master.getDigital(ControllerDigital::A) || partner.getDigital(ControllerDigital::A)) {
			drive->setState(state);
			master.setText(2, 0, state.str());
			partner.setText(2, 0, state.str());
			break;
		}
		printf("%s\n", state.str().c_str());
		master.setText(0, 0, state.x);
		master.setText(1, 0, state.y);
		partner.setText(2, 0, state.theta);
	}
}


void disabled() {}


void competition_initialize() {}


void tank_drive(Controller controller) {
	drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
}


void skills() {}


void left() {
	drive->setState({1.5_ft, 2_ft, 0_deg});
	drive->driveToPoint({3_ft, 6_ft});
	piston(frontClamp, true, true);
	if (frontBumper.isPressed()) {
		drive->driveToPoint({1.5_ft, 2_ft}, true);
	}
}


void right() {
	drive->setState({9_ft, 2_ft, 0_deg});
	drive->driveToPoint({9_ft, 6_ft});
	piston(frontClamp, true, true);
	if (frontBumper.isPressed()) {
		drive->driveToPoint({9_ft, 2_ft}, true);
	}
}


void left_middle() {
	drive->setState({1.5_ft, 2_ft, 0_deg});
	drive->driveToPoint({3_ft, 6_ft});
	piston(frontClamp, true, true);
	if (frontBumper.isPressed()) {
		drive->driveToPoint({1.5_ft, 2_ft}, true);
	}
	else {
		piston(frontClamp, true, false);
		drive->driveToPoint({3_ft, 4_ft}, true);
		drive->driveToPoint({6_ft, 6_ft});
		piston(frontClamp, true, true);
		drive->driveToPoint({1.5_ft, 2_ft}, true);
		drive->turnToAngle(0_deg);
	}
}


void right_middle() {
	drive->setState({9_ft, 2_ft, 0_deg});
	drive->driveToPoint({9_ft, 6_ft});
	piston(frontClamp, true, true);
	if (frontBumper.isPressed()) {
		drive->driveToPoint({9_ft, 2_ft}, true);
	}
	else {
		piston(frontClamp, true, false);
		drive->driveToPoint({9_ft, 4_ft}, true);
		drive->driveToPoint({6_ft, 6_ft});
		piston(frontClamp, true, true);
		drive->driveToPoint({11_ft, 3_ft}, true);

		piston(tilt, true, true);
		pros::delay(250);
		piston(backClamp, true, false);
		
		drive->driveToPoint({10_ft, 4_ft});
		drive->turnToAngle(0_deg);
		asyncLift->setTarget(50);
		intake.moveVelocity(600);
		drive->driveToPoint({10_ft, 6_ft});
		drive->driveToPoint({10_ft, 2_ft}, true);
	}
}


void middle_left() {
	drive->setState({1.5_ft, 2_ft, 0_deg});
	drive->driveToPoint({6_ft, 6_ft});
	piston(frontClamp, true, true);
	if (frontBumper.isPressed()) {
		drive->driveToPoint({1.5_ft, 2_ft}, true);
	}
	else {
		piston(frontClamp, true, false);
		drive->driveToPoint({4_ft, 4_ft}, true);
		drive->driveToPoint({3_ft, 6_ft});
		piston(frontClamp, true, true);
		if (frontBumper.isPressed()) {
			drive->driveToPoint({1.5_ft, 2_ft}, true);
		}
	}
}


void middle_right() {
	drive->setState({9_ft, 2_ft, 0_deg});
	drive->driveToPoint({6_ft, 6_ft});
	piston(frontClamp, true, true);
	if (frontBumper.isPressed()) {
		drive->driveToPoint({9_ft, 2_ft}, true);
	}
	else {
		piston(frontClamp, true, false);
		drive->driveToPoint({8_ft, 4_ft}, true);
		drive->driveToPoint({9_ft, 6_ft});
		piston(frontClamp, true, true);
		if(frontBumper.isPressed()) {
			drive->driveToPoint({9_ft, 2_ft}, true);
		}
	}
}


void swiper_left() {
	drive->setState({1.5_ft, 2_ft, 0_deg});
	piston(swiper, false, true);
	drive->driveToPoint({3_ft, 5_ft});
	drive->turnToAngle(90_deg);
	drive->driveToPoint({4_ft, 5_ft});
	piston(frontClamp, true, true);
	if(frontBumper.isPressed()) {
		drive->driveToPoint({1.5_ft, 2_ft}, true);
	}
	piston(swiper, false, false);
}


void swiper_right() {
	drive->setState({9_ft, 2_ft, 0_deg});
	piston(swiper, false, true);
	drive->driveToPoint({9_ft, 5_ft});
	drive->turnToAngle(-90_deg);
	drive->driveToPoint({8_ft, 5_ft});
	piston(frontClamp, true, true);
	if(frontBumper.isPressed()) {
		drive->driveToPoint({9_ft, 2_ft}, true);
	}
	piston(swiper, false, false);
}


void autonomous() {
	piston(backClamp, true, false);
	piston(frontClamp, true, false);
	
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
	else if(selector::auton == 7) { // swiper (Left)
		swiper_left();
	}
	else if(selector::auton == 8) { // swiper (Right)
		swiper_right();
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
	else if(selector::auton == -7) { // swiper (Left)
		swiper_left();
	}
	else if(selector::auton == -8) { // swiper (Right)
		swiper_right();
	}
	else if(selector::auton == 0){ //Skills
		skills();
	}
}


void opcontrol() {
	bool backClampToggle = false;
	bool frontClampToggle = false;
	bool swiperToggle = false;
	bool flapToggle = false;
	bool driverToggle = false;
	driver = master;

	while(true){
		tank_drive(driver);

		// Front Goal related stuff on the right hand
		// lift
		if(driver.getDigital(ControllerDigital::R1) || partner.getDigital(ControllerDigital::R1)) {
			lift.moveVelocity(100);
		}
		else if(driver.getDigital(ControllerDigital::R2) || partner.getDigital(ControllerDigital::R2)) {
			lift.moveVelocity(-100);
		}
		else {
			lift.moveVelocity(0);
		}
		// clamp
		if(driver.getDigital(ControllerDigital::Y) || partner.getDigital(ControllerDigital::Y)) {
			if (frontClampToggle) {
				frontClampToggle = false;
				piston(frontClamp, true, false);
				pros::delay(200);
			}
			else {
				frontClampToggle = true;
				piston(frontClamp, true, true);
				pros::delay(200);
			}
		}
		
		// MOGO stuff
		// intake
		intake_switcher(intake_toggle);
		// mogo grab and tilt
		if(driver.getDigital(ControllerDigital::right) || partner.getDigital(ControllerDigital::right)) {
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

		// MISC stuff
		// swiper
		if(driver.getDigital(ControllerDigital::down) || partner.getDigital(ControllerDigital::down)) {
			if (swiperToggle) {
				swiperToggle = false;
				piston(swiper, false, true);
				pros::delay(200);
			}
			else {
				swiperToggle = true;
				piston(swiper, false, false);
				pros::delay(200);
			}
		}
		// flap
		if(driver.getDigital(ControllerDigital::B) || partner.getDigital(ControllerDigital::B)) {
			if (flapToggle) {
				flapToggle = false;
				piston(flap, false, true);
				pros::delay(200);
			}
			else {
				flapToggle = true;
				piston(flap, false, false);
				pros::delay(200);
			}
		}
		// auto clamp
		if(partner.getDigital(ControllerDigital::A) && frontBumper.isPressed() && !frontClampToggle) {
				frontClampToggle = true;
				piston(frontClamp, true, true);
				pros::delay(200);
		}
		// override master controller
		if(partner.getDigital(ControllerDigital::X)) {
			if (driverToggle) {
				driverToggle = false;
				driver = master;
				intake_toggle = false;
				pros::delay(200);
			}
			else {
				driverToggle = true;
				driver = partner;
				intake_toggle = true;
				pros::delay(200);
			}
		}
		// safety features?
		if(partner.getDigital(ControllerDigital::up)) {

		}
		if(partner.getDigital(ControllerDigital::right)) {

		}

		// AUTOMATION
		// intake unjammer
		/*
		if(intake.getActualVelocity() < 20 && intake.getTargetVelocity() > 20 && intake_toggle) {
			intake.moveRelative(-600, 600);
		}
		*/
		pros::delay(10);
	}
}