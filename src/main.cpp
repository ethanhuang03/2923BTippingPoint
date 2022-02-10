#include "main.h"
#include "ARMS/api.h"

Controller controller;

std::shared_ptr<ChassisController> drive;

auto backRightDrive = Motor(1);
auto frontRightDrive = Motor(2);
auto backLeftDrive = Motor(-3);
auto frontLeftDrive = Motor(-4);

auto lift = Motor(5);
auto intake = Motor(6);
auto mogo = Motor(7);
auto clamp = Motor(8);

bool clamp_flag = 0;

auto RightDrive = MotorGroup({frontRightDrive, backRightDrive});
auto LeftDrive = MotorGroup({frontLeftDrive, backLeftDrive});

auto rightEncoder = IntegratedEncoder(backRightDrive);
auto leftEncoder = IntegratedEncoder(backLeftDrive);

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
	drive->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::rightY));
}

void arcade_drive(){
	pros::lcd::set_text(1, "Arcade Drive");
	drive->getModel()->arcade(controller.getAnalog(ControllerAnalog::leftY), controller.getAnalog(ControllerAnalog::leftX));
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

void R1(){
	if(button_R1.isPressed()){
		intake.moveVelocity(100);
	}
	else{
		intake.moveVelocity(0);
	}
}

void R2(){
	if(button_R2.isPressed()){
		intake.moveVelocity(-100);
	}
	else{
		intake.moveVelocity(0);
	}
}

void L1(){
	if(button_L1.isPressed()){
		mogo.moveVelocity(100);
	}
	else{
		mogo.moveVelocity(0);
	}
}

void L2(){
	if(button_L2.isPressed()){
		mogo.moveVelocity(-100);
	}
	else{
		mogo.moveVelocity(0);
	}
}

void X(){
	if(button_X.isPressed()){
		clamp_flag = 0;
		clamp.moveVelocity(-100);
	}
	else{
		clamp.moveVelocity(100);
	}
}

void Y(){
	if (clamp_flag) {
		clamp.moveVelocity(100);
	}
	if(button_Y.isPressed()){
		clamp_flag = 1;
		clamp.moveVelocity(100);
	}
	else{
		clamp.moveVelocity(0);
	}
}

void A(){
	if(button_A.isPressed()){
	}
	else{
	}
}

void B(){
	if(button_B.isPressed()){
	}
	else{
	}
}

void Up(){
	if(button_Up.isPressed()){
		lift.moveVelocity(100);
	}
	else{
		lift.moveVelocity(0);
	}
}

void Down(){
	if(button_Down.isPressed()){
		lift.moveVelocity(100);
	}
	else{
		lift.moveVelocity(0);
	}
}

void Left(){
	if(button_Left.isPressed()){
	}
	else{
	}
}

void Right(){
	if(button_Right.isPressed()){
	}
	else{
	}
}

void opcontrol() {
	pros::lcd::set_text(2, "User Control");
	while(true){
		tank_drive();
		R1();
		R2();
		L1();
		L2();
		X();
		Y();
		A();
		B();
		Up();
		Down();
		Left();
		Right();
		pros::delay(10);
	}

}
