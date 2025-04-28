#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/ext_adi.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"
#include "pros/gps.hpp"

#include <cstdio>
#include <string>


pros::MotorGroup left_motor_group({10, -9, -1}); // left motors on ports 1 (reversed), 2 (forwards), and 3 (reversed)
pros::MotorGroup right_motor_group({-20, 13, 15}); // right motors on ports 4 (forwards), 5 (reversed), and 6 (forwards)
pros::MotorGroup ladybrown({5,-11});
pros::Motor intake(19);
pros::Motor hooks(4);
pros::adi::DigitalOut clamp('E');
pros::adi::DigitalOut doink('C');
pros::adi::DigitalOut pistonLift('B');
pros::adi::DigitalOut tiplift('H');
pros::adi::DigitalOut doinkjr('B');
pros::Imu imu(17);
// horizontal tracking wheel encoder. Rotation sensor, port 20, not reversedx
//pros::Rotation horizontalEnc(13);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(-14);
pros::Rotation wallrotational(19);
pros::Rotation colorrot(-6);
// horizontal tracking wheel
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, lemlib::Omniwheel::NEW_2, -0.5);
// vertical tracking wheel
//lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, lemlib::Omniwheel::NEW_2, 1.8);
lemlib::TrackingWheel vertical_tracking_wheel(&verticalEnc, lemlib::Omniwheel::NEW_275, 1.375);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Optical color(7);
pros::adi::AnalogIn potentiometer ('A');
pros::GPS gps1(14);
// pros::gps_status_s_t status;

double x_pos;
double y_pos;
double x_offset = 0;
double y_offset = 0.1651;
    




bool clampbool = LOW;
bool pistonliftbool = LOW;
//bool doink = LOW;
bool R2_pressed = false;
bool A_pressed = false;
bool alliancecolor = false; //true = red alliance
bool B_pressed = false;
bool X_pressed = false;



// drivetrain setting
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              10.625, // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 0. If we had traction wheels, it would have been 8
);

lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, //&horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

lemlib::ControllerSettings lateral_controller(5.55, // proportional gain (kP) 6.9
                                              0.05, // integral gain (kI) 0.02
                                              20, // derivative gain (kD) 8
                                              7, // anti windup
                                              1, // small error range, in inches
                                              1400, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              700, // large error range timeout, in milliseconds
                                              80 // maximum acceleration (slew
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP) 2.5
                                              0, // integral gain (kI)
                                              20, // derivative gain (kD)
                                              10, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              5, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              80 // maximum acceleration (slew)
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
int states[numstates] = {0, 16, 27, 165};
int currstate = 0;
int target = 0;

void nextState(){ //macro to score
    currstate += 1;
    if(currstate == numstates){
        currstate = 0;
    }
    target = states[currstate];
}


void liftControl(){
    ladybrown.set_brake_mode(pros::MotorBrake::hold);
    double kp = 3;
    double error = target - (wallrotational.get_position()/100.0);
    if ((wallrotational.get_position()/100.0) > 340) {
       error = (360 + target) - (wallrotational.get_position()/100.0); 
    }
    double velocity = kp*error;
    // if(abs(error) > 10){
    //     intake.move(velocity);
    // }
    ladybrown.move(velocity);
    if((abs(error) > 5) && (abs(error) < 20)){
    intake.move(velocity);
    }
}

// void easyLoad(){
//     nextState();
//     pros::delay(700);
//     nextState();
//     nextState();
//     intake.move(-127);
//     pros::delay(1000);
//     nextState();
//     pros::delay(700);
//     nextState();
// }

void alliancedrive(){
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,2.3,500);
}

bool intakespin = false;
bool runintakejam = true;
// bool runfrontstage = false;
bool macropeck = false;
bool isextake = false;

void intakeunjam() {
    if (intakespin == true ) {
        intake.move(-127);
        if (intake.get_actual_velocity() == 0) {
            intake.move(127);
            pros::delay(50);
            intake.move(-127);
        }
    }
    else if (intakespin == false) {
        intake.move(0);
    }
    else if (intakespin == true && runintakejam == false) {
        intake.move(-127);
    } 
    // else if (intakespin == true && runfrontstage == true) {
    //     frontstage.move(-127);
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
            pros::delay(300);
            intake.move(0);
            pros::delay(200);
            intake.move(127);
            
            
        }
        //red ring color sort
        else if (intakespin == true && alliancecolor == false  && isextake == false && (color.get_hue() <= 40 || color.get_hue() >= 350)) {
            pros::lcd::print(6, "red ring detected");
            pros::delay(300);
            intake.move(0);
            pros::delay(200);
            intake.move(127);
        }
       
      
        
        else if (intakespin == true && runintakejam == true && macropeck == false && isextake == false) {
            intake.move(-127);
            if (intake.get_actual_velocity() == 0) {
                intake.move(127);
                pros::delay(220);
                intake.move(-127);
            }
        }
        else if (intakespin == false  && isextake == false) {
            intake.move(0);
        }
        else if (intakespin == true && runintakejam == false  && isextake == false) {
            intake.move(-127);
        } 
        // else if (intakespin == true && isextake == false) {
        //     hooks.move(0);
        //     stage.move(-127);front
        // } 
        else if (intakespin == true && macropeck == true && isextake == false) {
            intake.move(-127);
            if (intake.get_actual_velocity() == 0) {
                intake.move(127);
                pros::delay(25);
                intake.move(-127);
            }
        }
        else if (intakespin == false && isextake == true) {
            intake.move(50);
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

void redneg5(){
    alliancecolor = true;
            chassis.setPose(0,0,333);
            chassis.moveToPoint(-5,11,750,{.maxSpeed = 80});
            chassis.waitUntilDone();
            nextState();
            nextState();
            nextState();
            pros::delay(700);
            nextState();
            chassis.moveToPoint(9, -5, 500,{.forwards = false});
            chassis.moveToPoint(9, -20, 500, {.forwards = false});
            chassis.waitUntilDone();
            pros::delay(500);
            clamp.set_value(false);
            pros::delay(200);
            intakespin = true;
            chassis.turnToPoint(20,-34, 700);
            chassis.waitUntilDone();
            // doink.set_value(true);
            chassis.moveToPoint(20, -34, 750);
            chassis.turnToPoint(43, -40, 300);
            chassis.moveToPoint(43, -38, 850);
            chassis.moveToPoint(6, -25, 1100,{.forwards = false});
            chassis.waitUntilDone();
            // doink.set_value(false);
            chassis.turnToPoint(35, -25, 300);
            chassis.moveToPoint(35, -25, 650);
            chassis.turnToPoint(27, -5, 300);
            chassis.moveToPoint(27, -5, 750);
            chassis.waitUntilDone();
            runintakejam = false;
            chassis.turnToPoint(55, 25, 500);//corner
            chassis.moveToPoint(55, 25, 1000);
            chassis.turnToHeading(90,600);
            chassis.moveToPoint(65, 35, 800);
            chassis.moveToPoint(25, 0, 800, {.forwards = false, .maxSpeed = 40});
            chassis.waitUntilDone();
            runintakejam = true;
            tiplift.set_value(true);
            chassis.turnToPoint(-4, 5, 700);
            chassis.moveToPoint(-4, 5, 1100,{.maxSpeed = 80});
            chassis.turnToHeading(90, 500);
            chassis.moveToPoint(-20, 0, 1000, {.forwards = false});
            chassis.waitUntilDone();
            tiplift.set_value(false);
}




/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
}

int current_auton_selection = -1; // Initialize to an invalid value
const int numcases = 10;

const char* auton_names[numcases + 1] = {
    "redneg",
    "skills",
    "blueneg",
    "redpos",
    "bluepos",
    "SAWPred",
    "47skills",
    "SAWPblue",
    "Testing",
    "rushred",
    "rushblue"
};

// Function to update the selected autonomous mode efficiently
void update_auton_selector() {
    int pot_value = potentiometer.get_value(); // Read potentiometer value (10-4095)

    // Ensure potentiometer value is within valid range
    if (pot_value < 10) pot_value = 10;
    if (pot_value > 4095) pot_value = 4095;

    // Correct section size calculation
    int section_size = (4095 - 10 + numcases) / (numcases + 1);

    // Determine which autonomous mode to select
    current_auton_selection = (pot_value - 10) / section_size;

    // Ensure value stays within valid range
    if (current_auton_selection > numcases) current_auton_selection = numcases;  // Fixing boundary case

    // Print selected autonomous mode name (clearing only necessary lines)
    pros::lcd::clear_line(4);
    pros::lcd::print(4, "Auton: %s", auton_names[current_auton_selection]);

    // Overwrite previous text properly on the controller without flickering
    controller.print(0, 0, "Auton: %-10s", auton_names[current_auton_selection]); // Ensures old text is overwritten
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
    color.set_led_pwm(100);
    ladybrown.set_brake_mode(pros::MotorBrake::hold);
    gps1.set_offset(0.0, 165.1, 0.0);

    pros::Task liftControlTask([]{
        while(true) {
            // colorSort();
            liftControl();
            update_auton_selector();
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
 * If the robot is disabled or communications is los
 |t, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */ 

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
        case 0: //red negative
               alliancecolor = true;

            break;
        case 1: //Skills
            chassis.setPose(-3, 8, 180);
            alliancecolor = true;
            nextState();
            nextState();
            nextState();
            pros::delay(700);
            nextState();
            pros::delay(300);
            chassis.moveToPoint(20,15, 1250,{.forwards = false});
            chassis.waitUntilDone();
            clamp.set_value(false); // clamp first mogo
            intakespin = true;
            chassis.turnToPoint(22, 35, 650);
            chassis.moveToPoint(22, 35, 650);//ring one
            chassis.turnToPoint(30, 50, 350);
            chassis.moveToPoint(30, 50, 650);
            chassis.turnToPoint(47, 100, 550);
            chassis.waitUntilDone();
            nextState();
            runintakejam = false;
            chassis.moveToPoint(47, 95, 1150);// lb load
            
            chassis.moveToPoint(40, 69, 1100,{.forwards = false});
            chassis.waitUntilDone();
            target = 70;
            intakespin = false;
            isextake = true;
            pros::delay(100);
            isextake = false;
            intakespin = true;
            runintakejam = true;
            chassis.turnToPoint(65, 69, 650);
            chassis.moveToPoint(65, 69, 1800, {.maxSpeed = 40});
            chassis.waitUntilDone();
            nextState();
            nextState();
            chassis.moveToPoint(70, 69, 450);
            chassis.turnToHeading(120, 400);
            chassis.turnToHeading(60, 400);
            chassis.turnToHeading(90, 100);
            chassis.waitUntilDone();
            nextState();
            pros::delay(500);
            runintakejam = true;
            chassis.moveToPoint(47, 69, 650,{.forwards = false});
            chassis.turnToPoint(47, 10, 900);
            chassis.moveToPoint(47, 10, 2900,{.maxSpeed = 50});
            chassis.turnToHeading(90, 650);
            chassis.moveToPoint(70, 50, 900);
            chassis.moveToPoint(70, -20, 1050,{.forwards = false});
            chassis.waitUntilDone();
            intakespin = false;
            clamp.set_value(true);
            isextake = true;
            chassis.moveToPoint(0, 16, 700);
            chassis.waitUntilDone();
            isextake = false;
            chassis.turnToHeading(90, 900);


            
            chassis.moveToPoint(-25, 19, 1500, {.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            //clamp first mogo
            clamp.set_value(false);
            pros::delay(200);
           
            intakespin = true;
            chassis.turnToPoint(-20, 42, 750);  
            chassis.moveToPoint(-20, 42, 850); //ring 1
            chassis.turnToPoint(-32, 55, 300);
            chassis.moveToPoint(-32, 55, 600);
            chassis.turnToPoint(-42, 80, 400);
            chassis.waitUntilDone();
            // nextState();
            // runintakejam = false;
            chassis.moveToPoint(-42, 85, 600); //ring 2
            chassis.turnToPoint(-56, 110, 300);
            chassis.moveToPoint(-56, 110, 750);// ring 3
            pros::delay(1500);
            chassis.moveToPoint(-40, 80, 700, {.forwards = false}); 
            chassis.waitUntilDone();
          
            chassis.turnToPoint(-45, 50, 750);
            chassis.moveToPoint(-45, 50, 750);
            chassis.turnToPoint(-48, 2, 300);
            chassis.moveToPoint(-48, 2, 1900, {.maxSpeed = 45});
            chassis.moveToPoint(-45, 16, 600, {.forwards = false});
            chassis.turnToHeading(270, 800);
            pros::delay(1000);
            
            chassis.waitUntilDone();
            runintakejam = false;
            nextState();
            chassis.moveToPoint(-65, 40, 1100);
            chassis.moveToPoint(-70, -10, 1300, {.forwards = false, .maxSpeed = 70});//release first mogo
            chassis.waitUntilDone();
            clamp.set_value(true);
            intakespin = false;
            isextake = true;
            pros::delay(200);
            intakespin = true;
            isextake = false;
            chassis.turnToPoint(-43, 63, 500);
            chassis.moveToPoint(-43, 63, 1200, {.maxSpeed = 100});
            chassis.turnToPoint(-70, 67, 750, {.maxSpeed = 50});
            chassis.waitUntilDone();
            nextState();
            target = 70;
            intakespin = false;
            isextake = true;
            pros::delay(200);
            isextake = false;
            intakespin = true;
            chassis.moveToPoint(-70,66, 1250,{.maxSpeed = 90});
            pros::delay(2000);
            chassis.waitUntilDone();
            chassis.moveToPoint(-75, 66, 950);
            nextState();
            intakespin = false;
            isextake = true;
            chassis.waitUntilDone();
            chassis.moveToPoint(-70, 66, 700,{.forwards = false});
            chassis.moveToPoint(-80, 66, 900);
            
            
            pros::delay(500);
            isextake = false;
            //wallstake end
            //alliance and second mogo
            chassis.moveToPoint(-50, 70, 500, {.forwards = false});
            chassis.waitUntilDone();
            nextState();
            // chassis.waitUntilDone();
            // intakespin = true;
            // chassis.turnToHeading(90, 500);// Alliance ring
            // chassis.moveToPoint(-25, 90, 750);
            // pros::delay(1000);
            // intakespin = false;
            chassis.turnToHeading(190, 750);
            chassis.moveToPoint(-30, 120, 750,{.forwards = false});
            chassis.moveToPoint(-25, 135, 1000, {.forwards = false, .maxSpeed = 60});
            chassis.waitUntilDone();
            clamp.set_value(false);
            //clamp bluemogo
            chassis.turnToHeading(110, 500);
            chassis.waitUntilDone();
            clamp.set_value(true);
            chassis.moveToPoint(-110,150,1250, {.forwards = false});
            
           
            chassis.moveToPoint(-40, 115, 900,{.maxSpeed = 70});
            chassis.turnToHeading(270, 650);
            intakespin = false;
            chassis.moveToPoint(10, 115, 1200, {.forwards = false, .maxSpeed = 70});
            
            chassis.waitUntilDone();
            runintakejam = true;
            clamp.set_value(false);
            chassis.waitUntilDone();
            intakespin = true;
            chassis.turnToPoint(20, 90, 700);
            chassis.moveToPoint(20, 90, 700);
            chassis.turnToPoint(50, 115, 700);
            chassis.moveToPoint(50, 115, 650);
            chassis.turnToPoint(45, 120, 450);
            chassis.moveToPoint(45, 120, 650);
            chassis.turnToHeading(240, 1000);
            chassis.waitUntilDone();
            clamp.set_value(true);//unclmap third mogo
            chassis.moveToPoint(70, 130, 1250,{.forwards = false});
            
            chassis.waitUntilDone();
            intakespin = false;
            nextState();
            nextState();
            nextState();
            
            chassis.moveToPoint(50, 120, 650);
            chassis.turnToHeading(45, 650);
            chassis.moveToPoint(0, 70, 2000,{.forwards = false});
            chassis.waitUntilDone();
            nextState();
            // chassis.turnToPoint(-65, 113, 500);
            // chassis.moveToPoint(-65, 113, 2000, {.maxSpeed = 60});
            // pros::delay(1500);
            // chassis.waitUntilDone();
            // chassis.turnToPoint(0, 65, 900); 
            // chassis.moveToPoint(0, 65, 2000);
            // chassis.turnToPoint(25, 85, 750);
            // chassis.moveToPoint(25, 85, 750);
            // chassis.turnToPoint(40, 85, 500);
            // chassis.moveToPoint(40, 85, 1000);
            // chassis.turnToPoint(40, -20, 1000);
            // chassis.moveToPoint(40, -20, 5000, {.maxSpeed = 50});
            // chassis.turnToHeading(90, 750);
            // chassis.waitUntilDone();
            // nextState();
            // chassis.moveToPoint(60, 30, 1000);
            // chassis.turnToHeading(0, 500);
            // chassis.moveToPoint(65, -10, 750,{.forwards = false});
            // chassis.waitUntilDone();
            // runintakejam = false;
            // clamp.set_value(true);//unclamp 2nd mogo score
            // chassis.moveToPoint(40, 0, 1000);
            // chassis.turnToHeading(120, 750);
            // chassis.moveToPoint(17,13, 1000,{.forwards = false});
            // chassis.waitUntilDone();
            // clamp.set_value(false); // clamp third mogo
            // chassis.turnToPoint(17, 35, 800);
            // chassis.waitUntilDone();
            // target = 70;
            // intakespin = false;
            // isextake = true;
            // pros::delay(100);
            // isextake = false;
            // intakespin = true;
            // chassis.moveToPoint(17, 35, 1000);
            
            break;
        case 2: // blue negative
            alliancecolor = false;
            chassis.setPose(0,0,-333);
            chassis.moveToPoint(5.5,11,750,{.maxSpeed = 80});
            chassis.waitUntilDone();
            nextState();
            nextState();
            nextState();
            pros::delay(700);
            nextState();
            chassis.moveToPoint(-9, -5, 450,{.forwards = false});
            chassis.moveToPoint(-9, -20, 450, {.forwards = false});
            chassis.waitUntilDone();
            pros::delay(200);
            clamp.set_value(false);
            pros::delay(100);
            intakespin = true;
            chassis.turnToPoint(-20,-34, 700);
            chassis.waitUntilDone();
            // doink.set_value(true);
            chassis.moveToPoint(-20, -34, 750);
            chassis.turnToPoint(-43, -38, 300);
            chassis.moveToPoint(-43, -40, 850);
            chassis.moveToPoint(-10, -25, 700,{.forwards = false});
            chassis.waitUntilDone();
            chassis.turnToPoint(-35, -25, 300);
            chassis.moveToPoint(-35, -25, 650);
            chassis.turnToPoint(-27, -5, 300);
            chassis.moveToPoint(-27, -5, 750);
            chassis.waitUntilDone();
            runintakejam = false;
            chassis.turnToPoint(-55, 25, 500);//corner
            chassis.moveToPoint(-55, 25, 1000);
            chassis.turnToHeading(-90,600);
            chassis.moveToPoint(-65, 45, 800);
            chassis.moveToPoint(-25, 0, 800, {.forwards = false, .maxSpeed = 40});
            chassis.waitUntilDone();
            runintakejam = true;
            tiplift.set_value(true);
            chassis.turnToPoint(2, 4, 800);
            chassis.moveToPoint(2, 4, 1100,{.maxSpeed = 80});
            chassis.waitUntilDone();
            tiplift.set_value(false);
            chassis.turnToHeading(180, 500);
            nextState();
            nextState();
            nextState();
            chassis.moveToPoint(10, -30, 1500);


            break;           
        case 3: //red positive safe
            alliancecolor = true;
            chassis.setPose(0,0,-333);
            // chassis.moveToPoint(5,11,750,{.maxSpeed = 80});
            // chassis.waitUntilDone();
            // nextState();
            // nextState();
            // nextState();
            // pros::delay(700);
            // nextState();
            chassis.moveToPoint(-9, -5, 450,{.forwards = false});
            chassis.moveToPoint(-9, -20, 600, {.forwards = false});
            chassis.waitUntilDone();
            pros::delay(200);
            clamp.set_value(false);
            pros::delay(100);
            chassis.turnToPoint(8, -41, 700);
            
            // chassis.moveToPoint(8, -41, 700,{.maxSpeed = 60});
            // chassis.waitUntilDone();
        
            // doink.set_value(true);
            // pros::delay(300);
            chassis.moveToPoint(-40, 12, 1200,{.forwards = false, .maxSpeed = 110}); //notimeout change
            chassis.waitUntilDone();
            // doink.set_value(false);
            intakespin = true;
            chassis.turnToPoint(-30, -30, 450);
            
            chassis.moveToPoint(-30, -30, 750);
            chassis.turnToPoint(-45, -20,900);
            chassis.moveToPoint(-45, -20, 500);
            // chassis.turnToPoint(-45, -5, 400);

            // chassis.moveToPoint(-53, -5, 700);
            // chassis.turnToPoint(-53, 15, 300);
            // chassis.waitUntilDone();
            // doinkjr.set_value(true);
            
            // chassis.moveToPoint(-58, 15, 500);
            // chassis.turnToHeading(90, 900);
            // chassis.waitUntilDone();
            // doinkjr.set_value(false);
            // chassis.moveToPoint(-58, -10, 600,{.forwards = false});
            
            // chassis.moveToPoint(-58, 20, 1500);
            // chassis.moveToPoint(-40, -10, 500,{.forwards= false});
            // chassis.turnToHeading(90,500);
            // chassis.waitUntilDone();
            // clamp.set_value(true);
            // intakespin = false;
            // chassis.turnToHeading(0, 500);
            // chassis.moveToPoint(-40, -15, 500,{.forwards = false});
            break;
        case 4: //blue positive 
            alliancecolor = true;
            chassis.setPose(0,0,333);
            // chassis.moveToPoint(-5,11,750,{.maxSpeed = 80});
            // chassis.waitUntilDone();
            // nextState();
            // nextState();
            // nextState();
            // pros::delay(700);
            // nextState();
            chassis.moveToPoint(9, -5, 450,{.forwards = false});
            chassis.moveToPoint(9, -20, 600, {.forwards = false});
            chassis.waitUntilDone();
            pros::delay(200);
            clamp.set_value(false);
            pros::delay(100);
            chassis.turnToPoint(-8, -41, 700);
            
            chassis.moveToPoint(-8, -41, 700,{.maxSpeed = 60});
            chassis.waitUntilDone();
        
            doinkjr.set_value(true);
            pros::delay(300);
            chassis.moveToPoint(40, 12, 1200,{.forwards = false, .maxSpeed = 110}); //notimeout change
            chassis.waitUntilDone();
            doinkjr.set_value(false);
            intakespin = true;
            chassis.turnToPoint(30, -30, 450);
            
            chassis.moveToPoint(30, -30, 750);
            chassis.turnToPoint(45, -20,900);
            chassis.moveToPoint(45, -20, 500);
            chassis.turnToPoint(45, -5, 400);

            chassis.moveToPoint(53, -5, 700);
            chassis.turnToPoint(53, 15, 300);
            chassis.waitUntilDone();
            doink.set_value(true);
            
            chassis.moveToPoint(58, 15, 500);
            chassis.turnToHeading(-90, 900);
            chassis.waitUntilDone();
            doink.set_value(false);
            chassis.moveToPoint(58, -10, 600,{.forwards = false});
            
            chassis.moveToPoint(58, 20, 1500);
            chassis.moveToPoint(40, -10, 500,{.forwards= false});
            chassis.turnToHeading(-90,500);
            chassis.waitUntilDone();
            clamp.set_value(true);
            intakespin = false;
            chassis.turnToHeading(0, 500);
            chassis.moveToPoint(40, -15, 500,{.forwards = false});
            break;
        case 5: // SAWP red
            alliancecolor = true;
            chassis.setPose(0,0,333);
            chassis.moveToPoint(-5,11,750,{.maxSpeed = 80});
            chassis.waitUntilDone();
            nextState();
            nextState();
            nextState();
            pros::delay(700);
            nextState();
            chassis.moveToPoint(9, -5, 450,{.forwards = false});
            chassis.moveToPoint(9, -20, 750, {.forwards = false});
            chassis.waitUntilDone();
            
            clamp.set_value(false);
            pros::delay(100);
            intakespin = true;
            chassis.turnToPoint(20,-34, 600);
            chassis.waitUntilDone();
            // doink.set_value(true);
            chassis.moveToPoint(20, -34, 700);
            chassis.turnToPoint(48, -42, 300);
            chassis.moveToPoint(48, -40, 850);
            chassis.moveToPoint(10, -25, 700,{.forwards = false});
            chassis.waitUntilDone();
            // doink.set_value(false);
            chassis.turnToPoint(35, -25, 300);
            chassis.moveToPoint(35, -25, 900);
            chassis.waitUntilDone();
            tiplift.set_value(true);
            chassis.turnToPoint(-2, -2, 750);
            chassis.moveToPoint(-2, -2, 1900,{.maxSpeed = 80});
            chassis.waitUntilDone();
            tiplift.set_value(false);
            intakespin = false;
            clamp.set_value(true);
            chassis.turnToHeading(45, 750);
            chassis.moveToPoint(-35, -27, 1300,{.forwards = false});
            chassis.waitUntilDone();
            clamp.set_value(false);
            intakespin = true;
            chassis.turnToPoint(-60, -24, 750);
            chassis.moveToPoint(-60, -24, 800);
            chassis.turnToPoint(0, -30, 750);
            chassis.waitUntilDone();
            nextState();
            nextState();
            nextState();
            chassis.moveToPoint(0, -30, 1000,{.maxSpeed = 90});

            

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
            // runfrontstage = true;
            pros::delay(250);
            //center field
            chassis.moveToPoint(0, 65, 2000);
            chassis.moveToPoint(-20, 80, 1500);
            chassis.waitUntilDone();
            // runfrontstage = false;
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
            chassis.setPose(0,0,-333);
            chassis.moveToPoint(5.5,11,750,{.maxSpeed = 80});
            chassis.waitUntilDone();
            nextState();
            nextState();
            nextState();
            pros::delay(700);
            nextState();
            chassis.moveToPoint(-9, -5, 450,{.forwards = false});
            chassis.moveToPoint(-9, -20, 450, {.forwards = false});
            chassis.waitUntilDone();
            pros::delay(200);
            clamp.set_value(false);
            pros::delay(100);
            intakespin = true;
            chassis.turnToPoint(-20,-34, 700);
            chassis.waitUntilDone();
            // doink.set_value(true);
            chassis.moveToPoint(-20, -34, 750);
            chassis.turnToPoint(-43, -40, 300);
            chassis.moveToPoint(-43, -38, 850);
            chassis.moveToPoint(-10, -25, 700,{.forwards = false});
            chassis.waitUntilDone();
            // doink.set_value(false);
            chassis.turnToPoint(-35, -25, 300);
            chassis.moveToPoint(-35, -25, 900);
            chassis.waitUntilDone();
            tiplift.set_value(true);
            chassis.turnToPoint(2, -2, 800);
            chassis.moveToPoint(2, -2, 2000,{.maxSpeed = 80});
            chassis.waitUntilDone();
            tiplift.set_value(false);
            intakespin = false;
            clamp.set_value(true);
            chassis.turnToHeading(-45, 750);
            chassis.moveToPoint(45, -27, 1200,{.forwards = false});
            chassis.waitUntilDone();
            clamp.set_value(false);
            intakespin = true;
            chassis.turnToPoint(60, -19, 750);
            chassis.moveToPoint(60, -19, 800);
            chassis.turnToPoint(0, -30, 750);
            chassis.waitUntilDone();
            nextState();
            nextState();
            nextState();
            chassis.moveToPoint(10, -30, 1000);
            break;
        case 8: //testing


            clamp.set_value(false);
            alliancecolor = true;
            intakespin = true;
            
            break;
    
        case 9: //ringrushred
            chassis.setPose(0, 0, 341);
            clamp.set_value(true);
            alliancecolor = true;
            intakespin = true;
            runintakejam = true;
            
            chassis.moveToPoint(-14, 38, 2000,{.minSpeed = 40});
            pros::delay(500);
            doinkjr.set_value(true);
            chassis.moveToPoint(0, 20, 1000,{.forwards = false, .maxSpeed = 80});
            chassis.waitUntilDone();
            clamp.set_value(false);
            doinkjr.set_value(false);
            intakespin = true;
            chassis.turnToPoint(-25, 35, 750);
            chassis.moveToPoint(-25, 35, 1000);
        
        break;
        case 10: //ringrushblue

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
        pros::lcd::print(7, "lbpos:%ld", wallrotational.get_position());
        
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

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
		    alliancedrive();
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
        
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
		    if(X_pressed){
                doinkjr.set_value(false);
                X_pressed = false;
            } else{
                doinkjr.set_value(true);
                X_pressed = true;
            }
		}

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
		    if(B_pressed){
                tiplift.set_value(false);
                B_pressed = false;
            } else{
                tiplift.set_value(true);
                B_pressed = true;
            }
		}

        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
            nextState();
            nextState();
            nextState();
            pros::delay(700);
            nextState();
        }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)){
            chassis.setPose(0, 0, 0);
            chassis.turnToHeading(90, 10000);
        }
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, 10, 10000);
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
