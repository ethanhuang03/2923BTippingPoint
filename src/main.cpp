#include "main.h"
#include "autoSelect/selection.h"

Controller master(ControllerId::master);
Controller partner(ControllerId::partner);
Controller driver = master;

std::shared_ptr<ChassisController> drive;

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

ADIButton frontBumper('H');

pros::ADIDigitalOut backClamp('A');
pros::ADIDigitalOut tilt('B');
pros::ADIDigitalOut frontClamp('D');
pros::ADIDigitalOut swiper('C');

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


void right_auton_piston() {
	piston(frontClamp, true, false);
	pros::delay(890);
    piston(frontClamp, true, true);
}


void left_auton_piston() {
	piston(frontClamp, true, false);
	pros::delay(890);
	piston(frontClamp, true, true);
}


void middle_left_auton_piston() {
	piston(frontClamp, true, false);
	pros::delay(1190);
	piston(frontClamp, true, true);
}


void middle_right_auton_piston() {
	piston(frontClamp, true, false);
	pros::delay(1005);
	piston(frontClamp, true, true);
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
		.withDimensions({AbstractMotor::gearset::blue, (60/36)}, {{3.25_in, 37.8_cm}, imev5BlueTPR})
		.build();

	lift.setBrakeMode(AbstractMotor::brakeMode::hold);
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

void disabled() {}


void competition_initialize() {}


void tank_drive(Controller controller) {
	drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
}

void whole_drive(int velocity) {
	backRightDrive.moveVelocity(velocity);
	frontRightDrive.moveVelocity(velocity);
	topRightDrive.moveVelocity(velocity);
	backLeftDrive.moveVelocity(velocity);
	frontLeftDrive.moveVelocity(velocity);
	topLeftDrive.moveVelocity(velocity);
}

void left_drive(int velocity) {
	backLeftDrive.moveVelocity(velocity);
	frontLeftDrive.moveVelocity(velocity);
	topLeftDrive.moveVelocity(velocity);
}

void right_drive(int velocity) {
	backRightDrive.moveVelocity(velocity);
	frontRightDrive.moveVelocity(velocity);
	topRightDrive.moveVelocity(velocity);
}

void left() {
	pros::Task activate_pistons(left_auton_piston);
	whole_drive(600);
	pros::delay(800);
	whole_drive(-600);
	pros::delay(300);
	if (frontBumper.isPressed()) {
		while (topRightDrive.getActualVelocity() > -500) {
			pros::delay(10);
		}
		pros::delay(200);
		whole_drive(0);
	}
	else {
		whole_drive(600);
		pros::delay(50);
		whole_drive(0);
	}
}


void right() {
	pros::Task activate_pistons(right_auton_piston);
	whole_drive(600);
	pros::delay(800);
	whole_drive(-600);
	pros::delay(300);
	if (frontBumper.isPressed()) {
		while (topRightDrive.getActualVelocity() > -500) {
			pros::delay(10);
		}
		pros::delay(200);
		whole_drive(0);
	}
	else {
		whole_drive(600);
		pros::delay(50);
		whole_drive(0);
	}
}

void left_middle() {
	pros::Task activate_pistons(middle_left_auton_piston);
	whole_drive(600);
	pros::delay(1100);
	whole_drive(-600);
	pros::delay(300);
	if (frontBumper.isPressed()) {
		while (topRightDrive.getActualVelocity() > -500) {
			pros::delay(10);
		}
		pros::delay(200);
		whole_drive(0);
	}
	else {
		whole_drive(600);
		pros::delay(50);
		whole_drive(0);
	}
}

void right_middle() {
	pros::Task activate_pistons(middle_right_auton_piston);
	whole_drive(600);
	pros::delay(915);
	whole_drive(-600);
	pros::delay(300);
	if (frontBumper.isPressed()) {
		while (topRightDrive.getActualVelocity() > -500) {
			pros::delay(10);
		}
		pros::delay(200);
		whole_drive(0);
	}
	else {
		whole_drive(600);
		pros::delay(50);
		whole_drive(0);
	}
}


void swiper_left() {
	piston(swiper, false, true);
	whole_drive(600);
	pros::delay(700);
	right_drive(-400);
	pros::delay(600);
	whole_drive(0);
}


void swiper_right() {
	piston(swiper, false, true);
	whole_drive(600);
	pros::delay(500);
	left_drive(-400);
	pros::delay(600);
	piston(frontClamp, true, false);
	whole_drive(400);
	pros::delay(300);
	whole_drive(0);
	pros::delay(500);
	if (frontBumper.isPressed()) {
		piston(frontClamp, true, true);
		right_drive(-600);
		left_drive(600);
		pros::delay(80);
		whole_drive(-600);
		pros::delay(700);
		whole_drive(0);
		piston(swiper, false, false);
	}
}


void autonomous() {
	piston(backClamp, true, false);
	
	if(selector::auton == 1) { // Red Left
		left();
	}
	else if(selector::auton == 2) { // Red Right
		right();
	}
	else if(selector::auton == 3) { // Red Middle (From Left)
		left_middle();
	}
	else if(selector::auton == 4) { // Red Middle (From Right)
		right_middle();
	}
	else if(selector::auton == 5) { // swiper (Left)
		swiper_left();
	}
	else if(selector::auton == 6) { // swiper (Right)
		swiper_right();
	}
	else if(selector::auton == 1) { // Blue Left
		left();
	}
	else if(selector::auton == 2) { // Blue Right
		right();
	}
	else if(selector::auton == 3) { // Blue Middle (From Left)
		left_middle();
	}
	else if(selector::auton == 4) { // Blue Middle (From Right)
		right_middle();
	}
	else if(selector::auton == -5) { // swiper (Left)
		swiper_left();
	}
	else if(selector::auton == -6) { // swiper (Right)
		swiper_right();
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
				piston(frontClamp, true, true);
				pros::delay(200);
			}
			else {
				frontClampToggle = true;
				piston(frontClamp, true, false);
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
				pros::delay(450);
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

		pros::delay(10);
	}
}