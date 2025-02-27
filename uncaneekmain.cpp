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
pros::Motor ladybrown(5);
pros::MotorGroup intake({-20, 10});
pros::Motor frontstage(10);
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
bool alliancecolor = false; //true = red alliance





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
int states[numstates] = {0, 16, 28, 145};
int currstate = 0;
int target = 0;

void nextState(){ //macro to score
    currstate += 1;
    if(currstate == numstates){
        currstate = 0;
    }
    target = states[currstate];
}
bool bool360 = true;
void state360(){
    if(bool360){
        target = 230;
        bool360 = false;
    }
    else{
        currstate = 2;
        nextState();
        bool360 = true;
    }
    
}

void liftControl(){
    double kp = 2.1;
    double error = target - (wallrotational.get_position()/100.0);
    if ((wallrotational.get_position()/100.0) > 350) {
       error = (360 + target) - (wallrotational.get_position()/100.0); 
        pros::lcd::print(5, "hi");
    }
    double velocity = kp*error;
    if(abs(error) > 10){
        intake.move(velocity);
    }
    ladybrown.move(velocity);
}

void easyLoad(){
    nextState();
    pros::delay(700);
    nextState();
    nextState();
    intake.move(-127);
    pros::delay(1000);
    nextState();
    pros::delay(700);
    nextState();
}

bool intakespin = false;
bool runintakejam = true;
bool runfrontstage = false;
bool macropeck = false;
bool isextake = false;

void intakeunjam() {
    if (intakespin == true) {
        intake.move(-127);
        if (intake.get_actual_velocity() == 0) {
            intake.move(127);
            pros::delay(100);
            intake.move(-127);
        }
    }
    else if (intakespin == false) {
        intake.move(0);
    }
    else if (intakespin == true && runintakejam == false) {
        intake.move(-127);
    } 
    else if (intakespin == true && runfrontstage == true) {
        frontstage.move(-127);
    } 
    else if (intakespin == true && macropeck == true) {
        intake.move(-127);
        if (intake.get_actual_velocity() == 0) {
            intake.move(127);
            pros::delay(5);
            intake.move(-127);
        }
    } 
} //not used

void colorSortAuton(){
    if (pros::competition::is_autonomous()){
        intake.set_brake_mode(pros::MotorBrake::hold);
        //blue ring color sort
        if (intakespin == true && alliancecolor == true  && isextake == false && color.get_hue() >= 180 && color.get_hue() <= 240) {
            pros::lcd::print(6, "blue ring detected");
            intake.move(0);
            pros::delay(200);
            intake.move(-127);
        }
        //red ring color sort
        else if (intakespin == true && alliancecolor == false  && isextake == false && (color.get_hue() <= 40 || color.get_hue() >= 350)) {
            pros::lcd::print(6, "red ring detected");
            intake.move(0);
            pros::delay(200);
            intake.move(-127);
        }
        //intake unjam
        else if (intakespin == true && runintakejam == true && runfrontstage == false && macropeck == false && isextake == false) {
            intake.move(-127);
            if (intake.get_actual_velocity() == 0) {
                intake.move(127);
                pros::delay(150);
                intake.move(-127);
            }
        }
        else if (intakespin == false  && isextake == false) {
            intake.move(0);
        }
        else if (intakespin == true && runintakejam == false  && isextake == false) {
            intake.move(-127);
        } 
        else if (intakespin == true && runfrontstage == true && isextake == false) {
            frontstage.move(-127);
        } 
        else if (intakespin == true && macropeck == true && isextake == false) {
            intake.move(-127);
            if (intake.get_actual_velocity() == 0) {
                // intake.move(127);
                // pros::delay(25);
                intake.move(-127);
            }
        }
        else if (intakespin == false && isextake == true) {
            intake.move(127);
        } 
    }
}

void colorSortforRobot(){
    intake.set_brake_mode(pros::MotorBrake::hold);
    //alliance color true = red and false = blue
    if (alliancecolor == true && color.get_hue() >= 180 && color.get_hue() <= 240) {
        pros::lcd::print(6, "blue ring detected");
        intake.move(0);
        pros::delay(200);
        intake.move(-127);
    }
    else if (alliancecolor == false && (color.get_hue() <= 40 || color.get_hue() >= 350)) {
        pros::lcd::print(6, "red ring detected");
        intake.move(0);
        pros::delay(200);
        intake.move(-127);
    }
    else if (1) {
        intake.move(-127);
    }

} 

//not used




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
    color.set_led_pwm(40);
    ladybrown.set_brake_mode(pros::MotorBrake::hold);
    pros::Task liftControlTask([]{
        while(true) {
            // colorSort();
            liftControl();
            pros::delay(10);
        }
    });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    clamp.set_value(true);
}

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
int current_auton_selection = 5;
void autonomous() {
    clamp.set_value(true);
    pros::Task autotask([]{
        while(true) {
            colorSortAuton();
            // intakeunjam();
            pros::delay(5);
        }
    });
    switch(current_auton_selection){
        case 0: //red negative safe boltup
            alliancecolor = true;
            chassis.setPose(0, 0, 0);
            chassis.turnToHeading(300, 750,{.maxSpeed =55});
            chassis.moveToPoint(-10, 14, 1500, {.maxSpeed = 75});
            chassis.waitUntilDone();
            nextState();
            nextState(); //score on alliance stake
            pros::delay(1000);
            nextState();
            chassis.moveToPoint(0, 0, 1500, {.forwards = false, .maxSpeed = 90});
            chassis.turnToHeading(0, 750);
            chassis.moveToPoint(0, -18, 400, {.forwards = false});
            chassis.waitUntilDone();
            pros::delay(500);
            clamp.set_value(false); //clamp mogo
            pros::delay(500);
            intakespin = true;
            chassis.moveToPoint(0, -15, 300);
            chassis.turnToHeading(135, 650);
            chassis.moveToPoint(12, -29, 1000); //ring 1 on mogo
            chassis.turnToHeading(110,750);
            chassis.moveToPoint(22,-34,1000); //ring 2 on mogo
            chassis.moveToPoint(0, -20, 900,{.forwards = false});
            chassis.turnToHeading(90, 650);
            chassis.moveToPoint(15, -20, 1500); //ring 3 on mogo
            pros::delay(1750);
            chassis.waitUntilDone();
            chassis.turnToHeading(270,500);
            chassis.moveToPoint(-15, -10, 1000);
            nextState();
            nextState(); //touch ladder
            //intake alliance
            /*chassis.turnToHeading(270, 750);
            chassis.moveToPoint(-30, 0, 1000, {.maxSpeed = 75});
            chassis.waitUntilDone();
            intake.move(0);
            chassis.turnToHeading(0,750);
            chassis.moveToPoint(-30, 15, 750);
            chassis.turnToHeading(270, 750);
            chassis.moveToPoint(-10, 15, 1000);
            chassis.turnToHeading(180, 750);
            intake.move(-127);/*

            //corner clear
            /*intake.move(127);
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
            chassis.moveToPoint(40, 8, 1400, {.maxSpeed = 75});
            pros::delay(1000);
            chassis.moveToPoint(30, 12, 900, {.forwards = false, .maxSpeed = 75});
            chassis.turnToHeading(280,650);*/


            break;
        case 1: //Skills
            alliancecolor = true;
            chassis.setPose(0, 0, 0);
            clamp.set_value(true);
            runintakejam = false;
            intakespin = true;
            pros::delay(500);
            intakespin = false;
            chassis.moveToPoint(0, 13, 750);
            chassis.turnToHeading(90, 750);
            chassis.moveToPoint(-25, 16, 1250, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            //clamp first mogo
            clamp.set_value(false);
            pros::delay(200);
            runintakejam = true;
            intakespin = true;
            chassis.turnToHeading(0, 500);  
            chassis.moveToPoint(-25, 42, 900); //ring 1
            chassis.turnToHeading(300, 700);
            chassis.moveToPoint(-36, 55, 800);
            chassis.turnToHeading(355, 500);
            chassis.moveToPoint(-46, 86, 900); //ring 2
            chassis.waitUntilDone(); 
            chassis.moveToPoint(-42, 66, 1000, {.forwards = false}); //ring 3 68.5
            chassis.waitUntilDone();
            nextState();
            chassis.turnToHeading(271.5, 750);
            chassis.waitUntilDone();
            macropeck = true;
            chassis.waitUntilDone();

            chassis.moveToPoint(-61, 68.5, 1000, {.maxSpeed = 57});
            chassis.waitUntilDone();
            pros::delay(700); //removable
            intakespin = false;
            isextake = true;
            pros::delay(40);
            isextake = false;
            nextState();
            intake.set_brake_mode(pros::MotorBrake::coast);
            pros::delay(10);
            chassis.turnToHeading(275, 500);
            chassis.waitUntilDone();
            nextState();
            pros::delay(800);
            chassis.turnToHeading(270, 500);
            chassis.turnToHeading(275, 500);
            chassis.moveToPoint(-64, 64, 300);
            
            // right_motor_group.move(-50);
            // left_motor_group.move(-50);
            // pros::delay(300);
            // right_motor_group.move(50);
            // left_motor_group.move(50);
            // pros::delay(300);
            // right_motor_group.move(-80);
            // left_motor_group.move(-80);
            // pros::delay(300);

            chassis.waitUntilDone();
            chassis.waitUntilDone();


            macropeck = false;
            intake.set_brake_mode(pros::MotorBrake::hold);
            intakespin = true;
            nextState();
            chassis.waitUntilDone();
            chassis.moveToPoint(-49, 66, 1000, {.forwards = false});
            chassis.turnToHeading(181, 700);
            chassis.moveToPoint(-53, 12, 2000, {.maxSpeed = 80});
            chassis.moveToPoint(-51, 16, 1000, {.forwards = false});
            chassis.turnToHeading(270, 1000);
            chassis.moveToPoint(-59, 20, 1000);
            chassis.moveToPoint(-50, 20, 1000, {.forwards = false});
            chassis.turnToHeading(315, 500);
            chassis.moveToPoint(-65, 5, 1000, {.forwards = false});
            chassis.waitUntilDone();
            intakespin = false;
            pros::delay(100);
            clamp.set_value(true);
            //quadrant 1 end





            pros::delay(100);
            intakespin = true;
            chassis.moveToPoint(-50, 15, 1000);
            chassis.turnToHeading(270,1000);
            chassis.moveToPoint(0, 20, 3000,{.forwards = false, .minSpeed = 20});
            chassis.turnToHeading(270, 300);
            chassis.moveToPoint(20, 20, 1500,{.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            intakespin = false;
            pros::delay(100);





            //quadrant 2 start
            clamp.set_value(false);
            pros::delay(200);
            intakespin = true;
            //time short start
            chassis.turnToHeading(359,1000);
            chassis.moveToPoint(22,37,1000);
            chassis.turnToHeading(80, 900);
            chassis.waitUntilDone();
            chassis.moveToPoint(27,50,1000);
            chassis.turnToHeading(8, 500);
            chassis.moveToPoint(40, 88, 130);
            chassis.moveToPoint(37, 69, 900, {.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToHeading(90, 900);
            chassis.waitUntilDone();
            macropeck = true;
            nextState();
            chassis.moveToPoint(52, 70, 800, {.maxSpeed = 57});
            chassis.waitUntilDone();
            pros::delay(800);
            intakespin = false;
            isextake = true;
            pros::delay(40);
            isextake = false;
            nextState();
            chassis.turnToHeading(91, 500);
            chassis.waitUntilDone();
            nextState();
            pros::delay(700);
            chassis.turnToHeading(85, 400);
            chassis.turnToHeading(91, 400);
            chassis.waitUntilDone();
            nextState();
            intakespin = true;
            macropeck = false;
            chassis.moveToPoint(35, 66.5, 700, {.forwards = false});
            chassis.turnToHeading(180, 500);
            chassis.moveToPoint(38, 7, 1500, {.maxSpeed = 80});
            chassis.moveToPoint(38, 14, 800, {.forwards = false});
            chassis.moveToPoint(50, 18, 1000);
            chassis.turnToHeading(90, 500);
            chassis.moveToPoint(39, 18, 800, {.forwards = false});
            chassis.turnToHeading(55, 700);
            chassis.moveToPoint(54, 3, 1000, {.forwards = false});
            chassis.waitUntilDone();
            intakespin = false;
            pros::delay(100);
            clamp.set_value(true);

            //center field
            intakespin = true;
            runfrontstage = true;
            chassis.moveToPoint(-18.5, 70.5, 1200, {.minSpeed = 40});
            chassis.moveToPoint(-25, 84, 1500);
            chassis.waitUntilDone();
            chassis.turnToHeading(225, 1000);
            chassis.waitUntilDone();
            chassis.moveToPoint(-7, 107, 2000, {.forwards = false, .maxSpeed = 65});
            chassis.waitUntilDone();
            //last mogo for scoring
            clamp.set_value(false);
            pros::delay(200);
            intakespin = true;
            runfrontstage = false;
            chassis.moveToPoint(-64, 90, 1700);
            chassis.turnToHeading(0,1000);
            chassis.moveToPoint(-64, 108, 1000);
            chassis.turnToHeading(45, 500);
            chassis.moveToPoint(-60, 127, 1000);
            chassis.turnToHeading(325, 700);
            // chassis.turnToHeading(175, 500);
            // chassis.moveToPoint(-55, 100, 1000);
            chassis.moveToPoint(-75, 136, 1000,{.forwards = false});
            chassis.waitUntilDone();
            clamp.set_value(true);
            //release last mogo




            //chassis.moveToPoint(54, 67, 1000);
            // chassis.moveToPoint(42, 60, 1000, {.forwards = false});
            // chassis.waitUntilDone();
            // chassis.turnToHeading(180, 1000);
            // chassis.moveToPoint(44, 10, 5000, {.maxSpeed = 80});
            // chassis.moveToPoint(44, 20, 1000, {.forwards = false});
            // chassis.turnToHeading(90,1000);
            // chassis.moveToPoint(55, 25, 1000);
            // chassis.turnToHeading(0, 1000);
            // chassis.moveToPoint(60, 5, 1000, {.forwards = false});
            // chassis.turnToHeading(305, 750);
            // chassis.waitUntilDone();
            // intakespin = false;
            // pros::delay(100);

            // //quadrant 2 end
            // intakespin = true;
            // clamp.set_value(true);
            // runfrontstage = true;
            // pros::delay(100);





            // //center field
            // chassis.moveToPoint(-12.5, 72.5, 1200, {.minSpeed = 40});
            // chassis.moveToPoint(-27, 82, 1500);
            // chassis.waitUntilDone();
            // intakespin = false;
            // chassis.turnToHeading(225, 1000);
            // chassis.waitUntilDone();
            // intakespin = true;
            // chassis.moveToPoint(4, 120, 2000, {.forwards = false, .maxSpeed = 65});
            // chassis.waitUntilDone();
            // //last mogo for scoring
            // clamp.set_value(false);
            // pros::delay(200);
            // runfrontstage = false;
            // chassis.moveToPoint(-54, 90, 1700);
            // chassis.turnToHeading(0,1000);
            // chassis.moveToPoint(-59, 104, 1000);
            // chassis.turnToHeading(45, 500);
            // chassis.moveToPoint(-54, 118, 1000);
            // chassis.turnToHeading(175, 500);
            // chassis.moveToPoint(-54, 102, 1000);
            // chassis.moveToPoint(-70, 130, 1000,{.forwards = false});
            // chassis.waitUntilDone();
            // clamp.set_value(true);
            // //release last mogo




            // // //Quadrant 4
            // intakespin = false;
            // chassis.turnToHeading(135, 750);
            // chassis.moveToPoint(-40, 115, 7000, {.minSpeed = 70});
            // chassis.moveToPoint(-20, 115, 800, {.minSpeed = 40});
            // chassis.turnToHeading(60, 750, {.minSpeed = 40});
            // chassis.moveToPoint(25, 134, 1000, {.minSpeed = 40});
            // chassis.moveToPoint(55, 134,800, {.minSpeed = 20});
            // chassis.moveToPoint(-20,100, 1000, {.forwards = false});

            break;
        case 2: // boltup blue negative
            alliancecolor = false;
            chassis.setPose(0, 0, 0);
            chassis.turnToHeading(-300, 750,{.maxSpeed =55});
            chassis.moveToPoint(11.5, 13, 1500, {.maxSpeed = 75});
            chassis.waitUntilDone();
            nextState();
            nextState(); //score on alliance stake
            pros::delay(1000);
            nextState();
            chassis.moveToPoint(0, 0, 1500, {.forwards = false, .maxSpeed = 90});
            chassis.turnToHeading(0, 750);
            chassis.moveToPoint(1, -18, 400, {.forwards = false});
            chassis.waitUntilDone();
            pros::delay(500);
            clamp.set_value(false); //clamp mogo1
            pros::delay(500);
            intakespin = true;
            chassis.moveToPoint(0, -15, 300);
            chassis.turnToHeading(-135, 650);
            chassis.moveToPoint(-12, -28, 1000); //ring 1
            chassis.turnToHeading(-110,750);
            chassis.moveToPoint(-22,-32,1000); //ring 2
            chassis.moveToPoint(0, -20, 900,{.forwards = false});
            chassis.turnToHeading(-90, 650);
            chassis.moveToPoint(-15, -20, 1500); //ring 3
            pros::delay(1750);
            chassis.waitUntilDone();
            chassis.turnToHeading(-270,500);
            chassis.moveToPoint(15, -10, 1000);

            nextState();
            nextState(); //ladder touch
            break;

            
        case 3: //red positive
            alliancecolor = true;
            chassis.setPose(0, 0, 0);
            intakespin = true;
            runfrontstage = true;
            doink.set_value(true);
            chassis.moveToPoint(0, 32, 900, {.minSpeed = 50});
            chassis.turnToHeading(-300, 700);
            chassis.moveToPoint(-19, 24, 1200, {.forwards = false, .maxSpeed = 90});
            chassis.waitUntilDone();
            clamp.set_value(false);
            runfrontstage = false;
            doink.set_value(false);
            chassis.moveToPoint(0, 0, 1400, {.maxSpeed = 80});
            chassis.turnToHeading(83, 750, {.maxSpeed = 80});
            chassis.waitUntilDone();
            doink.set_value(true);
            chassis.moveToPoint(19, -3, 1000, {.maxSpeed = 80});
            chassis.turnToHeading(10, 750, {.maxSpeed = 80});
            chassis.waitUntilDone();
            doink.set_value(false);
            chassis.turnToHeading(86, 750, {.maxSpeed = 80});
            chassis.moveToPoint(26, -3, 1000, {.maxSpeed = 80});
            chassis.waitUntilDone();
            pros::delay(500);
            chassis.moveToPoint(0, 20, 1300, {.forwards = false});
            chassis.waitUntilDone();
            clamp.set_value(true);
            chassis.turnToHeading(180, 750);
            chassis.moveToPoint(6, 32, 1000, {.forwards = false});
            break;
            // chassis.moveToPoint(-13, 30, 1000, {.maxSpeed = 80});
            // chassis.turnToHeading(190, 1200, {.maxSpeed = 80});
            // doink.set_value(true);
            // chassis.moveToPoint(-29, 18, 2500, {.maxSpeed = 70});
            // chassis.turnToHeading(150, 800);
            // chassis.waitUntilDone();
            // doink.set_value(false);
            // chassis.turnToHeading(190, 800);
            // chassis.moveToPoint(-30.5, 10, 1000, {.maxSpeed = 70});
            // chassis.moveToPoint(-15, 18, 1000, {.maxSpeed = 70});
            // chassis.turnToHeading(0, 800);
            // chassis.waitUntilDone();
            // clamp.set_value(false);
            // chassis.turnToHeading(180, 800);
            // chassis.moveToPoint(-13, 30, 1200, {.forwards = false, .maxSpeed = 80});
            //break;
        case 4: //blue positive 
            alliancecolor = false; 
            chassis.setPose(0, 0, 0);
            intakespin = true;
            runfrontstage = true;
            doink.set_value(true);
            chassis.moveToPoint(0, 35, 900, {.minSpeed = 80});
            chassis.turnToHeading(300, 700);
            chassis.moveToPoint(17, 26, 1200, {.forwards = false, .maxSpeed = 90});
            chassis.waitUntilDone();
            doink.set_value(true);
            chassis.waitUntilDone();
            clamp.set_value(false);
            chassis.moveToPoint(-13, 30, 1000, {.maxSpeed = 80});
            chassis.waitUntilDone();
            runfrontstage = false;
            intakespin = false;
            chassis.turnToHeading(190, 1200, {.maxSpeed = 80});
            chassis.waitUntilDone();
            intakespin = true;
            doink.set_value(true);
            chassis.moveToPoint(-30, 12, 2500, {.maxSpeed = 70});
            chassis.waitUntilDone();
            intakespin = false;
            chassis.waitUntilDone();
            chassis.turnToHeading(110, 800);
            chassis.waitUntilDone();
            doink.set_value(false);
            chassis.moveToPoint(-30, 20, 1200, {.forwards = false}); //adjust
            chassis.turnToHeading(200, 800);
            intakespin = true;
            chassis.moveToPoint(-31.5, 5, 1000, {.maxSpeed = 70});
            chassis.moveToPoint(-15, 15, 1000, {.maxSpeed = 70});
            chassis.waitUntilDone();
            intakespin = false;
            chassis.turnToHeading(0, 800);
            chassis.waitUntilDone();
            clamp.set_value(true);
            chassis.turnToHeading(180, 800);
            chassis.moveToPoint(-13, 45, 2000, {.forwards = false, .maxSpeed = 80});
            break;
        case 5: // SAWP red
            alliancecolor = true;
            chassis.setPose(0, 0, 320);
            chassis.moveToPoint(-8.5, 10, 2000,{.maxSpeed = 80});
            chassis.waitUntilDone();
            nextState();
            nextState();
            nextState();
            pros::delay(750);
            nextState();
            chassis.turnToHeading(315, 500);
            chassis.moveToPoint(7, -10, 750, {.forwards = false, .maxSpeed = 75});
            chassis.moveToPoint(7, -25, 500, {.forwards = false, .maxSpeed = 75});
            chassis.waitUntilDone();
            pros::delay(250);
            clamp.set_value(false);
            // chassis.turnToHeading(130, 1000);
            // chassis.waitUntilDone();
            // intakespin = true;
            // chassis.moveToPoint(20, -33, 1500);
            // chassis.turnToHeading(150, 500);
            // chassis.moveToPoint(7, -17, 1250, {.forwards = false});
            // chassis.turnToHeading(90, 750);
            intakespin = true;
            chassis.turnToHeading(90, 500);
            chassis.moveToPoint(30, -19, 1500);
            chassis.turnToHeading(330, 750);
            chassis.moveToPoint(-7, 0, 800);
            chassis.turnToHeading(270, 600);
            chassis.waitUntilDone();
            clamp.set_value(true);
            chassis.moveToPoint(-45, 2, 3000, {.maxSpeed = 90});
            chassis.waitUntilDone();
            runfrontstage = true;
            chassis.turnToHeading(0, 750);
            chassis.moveToPoint(-37, -30, 750,{.forwards = false});
            chassis.waitUntilDone();
            pros::delay(500);
            clamp.set_value(false);
            runfrontstage = false;
    
            chassis.turnToHeading(270, 500);
            chassis.moveToPoint(-60, -20, 750);
            chassis.turnToHeading(90, 750);
            chassis.moveToPoint(-30, -20, 1000);
            chassis.waitUntilDone();
            nextState();
            nextState();
            nextState();



            // chassis.moveToPoint(0,3.75,500, {.minSpeed = 100});
            // chassis.turnToHeading(35, 500);
            // nextState();
            // nextState(); //scores ring 1 on alliance stake
            // pros::delay(450);
            // chassis.moveToPoint(-37, -2, 1250, {.forwards = false, .maxSpeed = 90});
            // chassis.waitUntilDone();
            // pros::delay(200);
            // clamp.set_value(false); //clamps mogo 1
            // nextState();
            // chassis.turnToHeading(240, 600);
            // intakespin = true;
            // chassis.moveToPoint(-49, -17, 900);
            // chassis.waitUntilDone();            
            // chassis.moveToPoint(-52, -25, 500,{.maxSpeed = 80,.minSpeed = 100}); //ring 2 on mogo 1
            // chassis.waitUntilDone();
            // chassis.moveToPoint(-57, -27, 500,{.maxSpeed = 80,.minSpeed = 100}); //ring 3 on mogo 1
            // chassis.turnToHeading(180, 300);
            // chassis.moveToPoint(-42, -7, 1000, { .forwards = false, .minSpeed = 100}); //ring 4 on mogo 1
            // chassis.moveToPoint(-39, -26, 1000);
            // chassis.turnToHeading(30, 500);
            // chassis.moveToPoint(-10, -5, 750);
            // chassis.turnToHeading(0, 500);
            // chassis.waitUntilDone();
            // clamp.set_value(true); //let go of mogo 1
            // chassis.moveToPoint(-10, 40, 1350, {.maxSpeed = 80}); //picks up ring 5
            // chassis.waitUntilDone();
            // intakespin = false;
            // chassis.turnToHeading(90, 500);
            // chassis.moveToPoint(-35, 45, 1000, {.forwards = false});
            // chassis.waitUntilDone();
            // clamp.set_value(false); //clamps mogo 2
            // chassis.turnToHeading(0, 750);
            // chassis.waitUntilDone();
            // intakespin = true; //ring 5 on mogo 2
            // chassis.moveToPoint(-40, 65, 750,{.minSpeed = 110}); //ring 6 on mogo 2
            // chassis.turnToHeading(180, 500);
            // chassis.waitUntilDone();
            // chassis.moveToPoint(-45,40, 750);
            // chassis.waitUntilDone();
            // nextState();
            // nextState(); //touch ladder



            break; 
        case 6: //47skills
            alliancecolor = true;
            chassis.setPose(0, 0, 0);
            intakespin = true;
            pros::delay(750);
            intakespin = false;
            chassis.moveToPoint(0, 13, 750);
            chassis.turnToHeading(90, 750);
            chassis.moveToPoint(-23, 17, 1250, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            //clamp first mogo
            clamp.set_value(false);
            pros::delay(250);
            intakespin = true;
            chassis.moveToPoint(-20,17,750);
            chassis.turnToHeading(0 , 500);  
            chassis.moveToPoint(-20, 45, 500);
            chassis.turnToHeading(300, 1000);
            chassis.moveToPoint(-55, 65, 1000);
            chassis.turnToHeading(270, 1000);
            chassis.moveToPoint(-48, 65, 1000, {.forwards = false});
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(-48, 12, 5000, {.maxSpeed = 80});
            chassis.moveToPoint(-48, 20, 1000, {.forwards = false});
            chassis.turnToHeading(270, 1000);
            chassis.moveToPoint(-59, 24, 1000);
            chassis.moveToPoint(-50, 20, 1000, {.forwards = false});
            chassis.turnToHeading(315, 500);
            chassis.moveToPoint(-65, 5, 1000, {.forwards = false});
            chassis.waitUntilDone();
            intakespin = false;
            pros::delay(250);
            clamp.set_value(true);


            //quadrant 1 end
            pros::delay(250);
            intakespin = true;
            chassis.moveToPoint(-50, 15, 1000);
            chassis.turnToHeading(270,1000);
            chassis.moveToPoint(0, 18, 3000,{.forwards = false});
            chassis.moveToPoint(20, 18, 1500,{.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            intakespin = false;
            pros::delay(250);
            //quadrant 2 start
            clamp.set_value(false);
            pros::delay(250);
            intakespin = true;
            //time short start
            chassis.turnToHeading(0,750);
            chassis.moveToPoint(20,40,1500);
            chassis.turnToHeading(60, 750);
            chassis.moveToPoint(45, 60, 1000);
            chassis.turnToHeading(90, 750);
            //time short
            chassis.turnToHeading(180, 1000);
            chassis.moveToPoint(47, 10, 5000, {.maxSpeed = 80});
            chassis.moveToPoint(43, 20, 1000, {.forwards = false});
            chassis.turnToHeading(90,1000);
            chassis.moveToPoint(55, 25, 1000);
            chassis.turnToHeading(0, 1000);
            chassis.moveToPoint(60, 5, 1000, {.forwards = false});
            chassis.turnToHeading(305, 750);
            chassis.waitUntilDone();
            intakespin = false;
            pros::delay(250);

            //quadrant 2 end
            clamp.set_value(true);
            intakespin = true;
            runfrontstage = true;
            pros::delay(250);
            //center field
            chassis.moveToPoint(0, 65, 2000);
            chassis.moveToPoint(-20, 80, 1500);
            chassis.waitUntilDone();
            runfrontstage = false;
            intakespin = false;
            chassis.moveToPoint(-25, 85, 1000);
            chassis.moveToPoint(-20, 80, 1500,{.forwards = false});
            chassis.turnToHeading(225, 1000);
            chassis.moveToPoint(0, 120, 2000, {.forwards = false, .maxSpeed = 65});
            chassis.waitUntilDone();
            //last mogo for scoring
            clamp.set_value(false);
            pros::delay(500);
            intakespin = true;
            chassis.moveToPoint(-60, 90, 1500);
            chassis.turnToHeading(0,1000);
            chassis.moveToPoint(-65, 110, 1000);
            chassis.turnToHeading(45, 500);
            chassis.moveToPoint(-55, 120, 1000);
            chassis.turnToHeading(190, 750);
            chassis.moveToPoint(-55,110, 1000);
            chassis.waitUntilDone();
            pros::delay(1000);
            chassis.moveToPoint(-75, 130, 1000,{.forwards = false});
            clamp.set_value(true);
            //release last mogo

            //Quadrant 4
            intakespin = false;
            chassis.turnToHeading(135, 750);
            chassis.moveToPoint(-40, 115, 1000, {.minSpeed = 70});
            chassis.moveToPoint(-20, 115, 800, {.minSpeed = 40});
            chassis.turnToHeading(60, 750, {.minSpeed = 40});
            chassis.moveToPoint(25, 134, 800, {.minSpeed = 40});
            chassis.moveToPoint(55, 134,800, {.minSpeed = 20});
            chassis.moveToPoint(-20,100, 800, {.forwards = false});
            break;        
        case 7: // SAWP blue
            alliancecolor = false;
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0,3.75,500, {.minSpeed = 100});
            chassis.turnToHeading(-35, 500);
            nextState();
            nextState();
            pros::delay(450);
            chassis.moveToPoint(37, -2, 1250, {.forwards = false, .maxSpeed = 90});
            chassis.waitUntilDone();
            pros::delay(200);
            clamp.set_value(false);
            nextState();
            chassis.turnToHeading(-240, 600);
            intakespin = true;
            chassis.moveToPoint(47, -17, 900);
            chassis.waitUntilDone();            
            chassis.moveToPoint(50, -25, 500,{.maxSpeed = 80,.minSpeed = 100});
            chassis.waitUntilDone();
            chassis.moveToPoint(55, -27, 500,{.maxSpeed = 80,.minSpeed = 100});
            chassis.turnToHeading(-180, 300);
            chassis.moveToPoint(42, -7, 1000, { .forwards = false, .minSpeed = 100});
            chassis.moveToPoint(39, -26, 1000);
            chassis.turnToHeading(-30, 500);
            chassis.moveToPoint(10, -5, 750);
            chassis.turnToHeading(-0, 500);
            chassis.waitUntilDone();
            clamp.set_value(true);
            chassis.moveToPoint(15, 40, 1350, {.maxSpeed = 80});
            chassis.waitUntilDone();
            intakespin = false;
            chassis.turnToHeading(-90, 500);
            chassis.moveToPoint(35, 40, 1000, {.forwards = false});
            chassis.waitUntilDone();
            clamp.set_value(false);
            chassis.turnToHeading(-0, 750);
            chassis.waitUntilDone();
            intakespin = true;
            chassis.moveToPoint(40, 65, 750,{.minSpeed = 110});
            chassis.turnToHeading(-180, 500);
            chassis.waitUntilDone();
            chassis.moveToPoint(45,40, 750);
            chassis.waitUntilDone();
            nextState();
            nextState();



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
    // state360();
    // clamp.set_value(true);

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
        
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			intake.move(127); 
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			intake.move(-127);
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

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
            state360();
        }
        
         if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
            pros::Task autoLoad{[] {
                easyLoad();
            }};
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
