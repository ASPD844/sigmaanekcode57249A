#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include <cstdio>


pros::MotorGroup left_motor_group({-2, -3, -4}); // left motors on ports 1 (reversed), 2 (forwards), and 3 (reversed)
pros::MotorGroup right_motor_group({16, 15, 14}); // right motors on ports 4 (forwards), 5 (reversed), and 6 (forwards)
pros::Motor ladybrown(11);
pros::MotorGroup intake({-20, 10});
pros::adi::DigitalOut clamp('E');
pros::adi::DigitalOut doink('C');
pros::adi::DigitalOut pistonLift('B');
pros::Imu imu(17);
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversedx
pros::Rotation horizontalEnc(13);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-18);
pros::Rotation wallrotational(19);
// horizontal tracking wheel
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, lemlib::Omniwheel::NEW_2, -0.5);
// vertical tracking wheel
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, lemlib::Omniwheel::NEW_275, 1.8);
lemlib::TrackingWheel vertical_tracking_wheel(&verticalEnc, lemlib::Omniwheel::NEW_275, -1.5);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Optical color(12);
bool clampbool = LOW;
bool pistonliftbool = LOW;
//bool doink = LOW;
bool R2_pressed = false;
bool A_pressed = false;
bool alliancecolor = true; //true = red alliance





// drivetrain setting
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              11.3125, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              480, // drivetrain rpm is 360
                              0 // horizontal drift is 0. If we had traction wheels, it would have been 8
);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

lemlib::ControllerSettings lateral_controller(5.45, // proportional gain (kP) 6.9
                                              0.05, // integral gain (kI) 0.02
                                              20, // derivative gain (kD) 8
                                              3, // anti windup
                                              1, // small error range, in inches
                                              1400, // small error range timeout, in milliseconds
                                              5, // large error range, in inches
                                              700, // large error range timeout, in milliseconds
                                              30 // maximum acceleration (slew
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2.5, // proportional gain (kP) 2.5
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              5, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              70 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127 
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, //  lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve, 
                        &steer_curve
);

const int numstates = 4;
int states[numstates] = {0, 14, 29, 145};
int currstate = 0;
int target = 0;

void nextState(){ //macro to score
    currstate += 1;
    if(currstate == 4){
        currstate = 0;
    }
    target = states[currstate];
}

void liftControl(){
    double kp = 2.5;
    double error = target - (wallrotational.get_position()/100.0);
    double velocity = kp*error;
    ladybrown.move(velocity);
}


void colorSort(){
    if (alliancecolor == true) {
        if (color.get_hue() >= 140 && color.get_hue() <= 300) {
            pros::lcd::print(6, "blue ring detected");;
            intake.move(0);
            intake.move(20);
        }
    }
    if (alliancecolor == false) {
        if (color.get_hue() <= 50 || color.get_hue() >= 300) {
            pros::lcd::print(6, "red ring detected");
            intake.move(0);
            intake.move(20);
        }
    }
}




/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    ladybrown.set_brake_mode(pros::MotorBrake::hold);
    pros::Task liftControlTask([]{
        while(true) {
            liftControl();
            colorSort();
            pros::delay(10);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */ 

int current_auton_selection = 1;
void autonomous() { 
    switch(current_auton_selection){
        case 0: //SAFE RINGS SIDE
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, -18, 400, {.forwards = false});
            chassis.waitUntilDone();
            pros::delay(500);
            clamp.set_value(true);
            intake.move(-127);
            chassis.moveToPoint(0, -15, 300);
            chassis.turnToHeading(135, 650);
            chassis.moveToPoint(15, -30, 900);
            chassis.turnToHeading(120,650);
            chassis.moveToPoint(23,-34,1000);
            chassis.moveToPoint(0, -20, 900,{.forwards = false});
            chassis.turnToHeading(90, 650);
            chassis.moveToPoint(15, -20, 900);
            chassis.waitUntilDone();
            pros::delay(750);
            intake.move(127);
            chassis.moveToPoint(25, -20, 400);
            chassis.moveToPoint(15, -20, 400,{.forwards = false});
            chassis.turnToHeading(45, 650);
            intake.move(-127);
            chassis.moveToPoint(30, 0, 900);
            doink.set_value(true);
            chassis.turnToHeading(20,400);
            chassis.moveToPoint(40, 8, 900,{.maxSpeed = 75});
            chassis.turnToHeading(270,650);
            chassis.waitUntilDone();
            doink.set_value(false);
            chassis.turnToHeading(30,650);
            chassis.moveToPoint(45, 13, 1400, {.maxSpeed = 75});
            chassis.moveToPoint(40, 8, 900, {.forwards = false, .maxSpeed = 75});
            chassis.turnToHeading(280,650);


            break;
        case 1: //AneekSkills
            chassis.setPose(0, 0, 0);
            intake.move(-127);
            pros::delay(750);
            intake.move(0);
            chassis.moveToPoint(0, 13, 750);
            chassis.turnToHeading(90, 750);
            chassis.moveToPoint(-30, 17, 750, {.forwards = false, .maxSpeed = 100});
            chassis.waitUntilDone();
            clamp.set_value(true);
            pros::delay(250);
            intake.move(-127);
            chassis.turnToHeading(0 , 500);  
            chassis.moveToPoint(-30, 45, 1000);
            chassis.turnToHeading(60, 1000);
            chassis.moveToPoint(-55, 65, 1000);
            chassis.turnToHeading(270, 1000);
            chassis.moveToPoint(-48, 65, 1000, {.forwards = false});
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-48, 10, 5000, {.maxSpeed = 80});
            chassis.moveToPoint(-48, 20, 1000, {.forwards = false});
            chassis.turnToHeading(270, 1000);
            chassis.moveToPoint(-59, 24, 1000);
            chassis.moveToPoint(-50, 20, 1000, {.forwards = false});
            chassis.turnToHeading(45, 1000);
            chassis.moveToPoint(-65, 5, 1000, {.forwards = false});
            chassis.waitUntilDone();
            intake.move(127);
            pros::delay(250);
            clamp.set_value(false);
            pros::delay(250);
            intake.move(-127);
            chassis.moveToPoint(-50, 15, 1000);
            chassis.turnToHeading(270,1000);
            chassis.moveToPoint(0, 20, 3000,{.forwards = false});
            chassis.moveToPoint(20, 20, 1500,{.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            clamp.set_value(true);
            chassis.turnToHeading(0,1000);
            chassis.moveToPoint(20,35,1000);
            chassis.turnToHeading(60, 1000);
            chassis.moveToPoint(50, 70, 1000);
            chassis.turnToHeading(90, 1000);
            chassis.moveToPoint(43, 70, 1000);
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(43, 15, 5000, {.maxSpeed = 80});
            chassis.moveToPoint(43, 20, 1000, {.forwards = false});
            chassis.turnToHeading(90,1000);
            chassis.moveToPoint(55, 25, 1000);
            chassis.turnToHeading(0, 1000);
            chassis.moveToPoint(60, 5, 1000, {.forwards = false});
            chassis.turnToHeading(315, 500);
            chassis.waitUntilDone();
            intake.move(127);
            pros::delay(250);
            clamp.set_value(false);
            intake.move(0);
            pros::delay(250);
            chassis.moveToPoint(0, 65, 2000);
            chassis.moveToPoint(-30, 80, 1000);
            chassis.turnToHeading(225, 1000);
            chassis.moveToPoint(0, 120, 2000, {.forwards = false, .maxSpeed = 65});
            chassis.waitUntilDone();
            clamp.set_value(true);
            intake.move(-127);
            chassis.moveToPoint(-65, 90, 1000);
            chassis.turnToHeading(0,1000);
            chassis.moveToPoint(-65, 100, 1000);
            chassis.turnToHeading(45, 500);
            chassis.moveToPoint(-55, 115, 1000);
            chassis.turnToHeading(180, 750);
            chassis.moveToPoint(-55, 100, 1000);
            
            



            break;
  
        case 2: // 10/11 matchauto red negativeide
            chassis.setPose(0,0,0);
            chassis.moveToPoint(0, -28, 1000, {.forwards = false}); //mogo 1
            chassis.waitUntilDone();
            clamp.set_value(true);
            pros::delay(300);
            intake.move(-127); //spins intake to score ring 1 (preload)
            chassis.turnToHeading(90, 800);
            chassis.moveToPoint(30, -28, 1000); //ring 2
            break;

 
    }
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    pros::delay(20);
    while (true) {
         // print robot location to the brain screen
        pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
        pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
        pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
        pros::lcd::print(7, "pos:%ld", wallrotational.get_position());
        
        // get left y and right x positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        

        //move ladybrown mech
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){
            nextState();
        }
        /*if(color.get_hue()>200){
            pros::Task task{[]{
                color_sort();
            }}
        }*/
        



        //move intake (NOT TOGGLE | MUST HOLD)
        
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake.move(-127);    
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			intake.move(127);
        } else{
            intake.move(0);
            pros::lcd::print(6, "");
        }

        // mogo mech controls
		if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
		    if(R2_pressed){
                clamp.set_value(false);
                R2_pressed = false;
            } else{
                clamp.set_value(true);
                R2_pressed = true;
            }
		}
        
        //piston lift controls
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
		    if(A_pressed){
                doink.set_value(false);
                A_pressed = false;
            } else{
                doink.set_value(true);
                A_pressed = true;
            }
		}

        // BOT MOVEMENT!!!!!!!

        double ctrX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); 
        double ctrY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

        double pwrF = ctrY;
        double pwrT = ctrX;

        if(1){
            double pwrF3 = pwrF*pwrF*pwrF;
            pwrF3 = pwrF3/10000;
        }

        if(1){
            double pwrT3 = pwrT*pwrT*pwrT;
            pwrT3 = pwrT3/10000;
            pwrT*=.925; //makes turning less sensitive
        }

        double pwrL = pwrF+pwrT;
        double pwrR = pwrF-pwrT;
        left_motor_group.move(pwrL);
        right_motor_group.move(pwrR);

        // delay to save resources
        pros::delay(25);
    }
}
