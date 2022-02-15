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

auto rightEncoder = IntegratedEncoder(backRightDrive);
auto leftEncoder = IntegratedEncoder(backLeftDrive);

void initialize() {
	/*Logger::setDefaultLogger( //log output to pros terminal
		std::make_shared<Logger>(
			TimeUtilFactory::createDefault().getTimer(),
			"/ser/sout",
			Logger::LogLevel::debug
		)
	);*/
	pros::lcd::initialize();
	selector::init();
	pros::lcd::set_text(0, "King's B | 2923B");
	drive = ChassisControllerBuilder()
		.withMotors(LeftDrive, RightDrive)
		//Green gearset, 4 in wheel diam, 11.5 in wheel track
		.withDimensions(AbstractMotor::gearset::green, {{4_in, 15_in}, imev5GreenTPR})
		.withSensors(leftEncoder, rightEncoder) //.withSensors(leftADI_Encoder, rightADI_Encoder)
		.withOdometry() // or .withOdometry({{4_in, 15_in}, quadEncoderTPR})
		.withGains(
			{0, 0, 0}, // Distance controller gains
			{0, 0, 0}, // Turn controller gains
			{0, 0, 0}  // Angle controller gains (helps drive straight)
		)
		.buildOdometry();
}

void disabled() {}

void competition_initialize() {}

void tank_drive(){
	pros::lcd::set_text(1, "Tank Drive");
	drive->getModel()->tank(master.getAnalog(ControllerAnalog::leftY), master.getAnalog(ControllerAnalog::rightY));
}

void arcade_drive(){
	pros::lcd::set_text(1, "Arcade Drive");
	drive->getModel()->arcade(master.getAnalog(ControllerAnalog::leftY), master.getAnalog(ControllerAnalog::leftX));
}

void red_front(){
	drive->moveDistance(12_in);
	drive->turnAngle(90_deg);
}

void red_back(){

}

void do_nothing(){

}

void blue_front(){

}

void blue_back(){

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
		if(partner.getDigital(ControllerDigital::R1)){
			intake.moveVelocity(-100);
		}
		else if(partner.getDigital(ControllerDigital::R2)){
			intake.moveVelocity(100);
		}
		else{
			intake.moveVelocity(0);
		}

		if(partner.getDigital(ControllerDigital::L1)){
			mogo.moveVelocity(100);
		}
		else if(partner.getDigital(ControllerDigital::L1)){
			mogo.moveVelocity(-100);
		}
		else{
			mogo.moveVelocity(0);
		}

		if(partner.getDigital(ControllerDigital::X)){
			clamp.moveVelocity(-100);
		}
		else if(partner.getDigital(ControllerDigital::Y)){
			clamp.moveVelocity(100);
		}
		else{
			clamp.moveVelocity(0);
		}

		if(partner.getDigital(ControllerDigital::up)){
			lift.moveVelocity(100);
		}
		else if(partner.getDigital(ControllerDigital::down)){
			lift.moveVelocity(-100);
		}
		else{
			lift.moveVelocity(0);
		}

		pros::delay(10);
	}
}
