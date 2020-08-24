//MASKED ROBOTICS HOMONOMIC DRIVE CODE BASE
//NMACERI ET AL
#include "vex.h"

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Motor1               motor         1               
// Motor10              motor         10              
// Motor11              motor         11              
// Motor20              motor         20              
// ---- END VEXCODE CONFIGURED DEVICES ----
#include <algorithm>
#include <cmath>
using namespace vex;

//#region config_globals
vex::motor      back_right_motor(vex::PORT1, vex::gearSetting::ratio36_1, true);
vex::motor      back_left_motor(vex::PORT10, vex::gearSetting::ratio36_1, false);
vex::motor      front_right_motor(vex::PORT11, vex::gearSetting::ratio36_1, true);
vex::motor      front_left_motor(vex::PORT20, vex::gearSetting::ratio36_1, false);
vex::controller con(vex::controllerType::primary);
//#endregion config_globals

//variables regarding conversion from inches to degrees
const double WHEEL_DIAMETER=4.0; 
const float WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER*3.14156;
const float INCHES_PER_DEGREE= WHEEL_CIRCUMFERENCE/360;
const double WHEEL_OFFSET = sqrt(2); 
const double TURNING_DIAMETER = 20.75; 
const float TURNING_RATIO= TURNING_DIAMETER/WHEEL_DIAMETER;
const float ROTATION_OFFSET=1.0; 
const int power = 50;

//this method will move the robot X amount of inches specified
void move(int option, float inches){
  //determines necessary about of degrees to move the robot
  float degrees = inches/INCHES_PER_DEGREE;
  switch(option){
  //Switch to determine how robot should behave with options
  //case 0 moves robot up and down
  case 0:
    back_right_motor.startRotateFor((degrees*WHEEL_OFFSET), vex::rotationUnits::deg, power, vex::velocityUnits::pct);
    back_left_motor.startRotateFor(degrees*WHEEL_OFFSET, vex::rotationUnits::deg, power, vex::velocityUnits::pct);
    front_right_motor.startRotateFor((degrees*WHEEL_OFFSET), vex::rotationUnits::deg, power, vex::velocityUnits::pct);
    front_left_motor.startRotateFor(degrees*WHEEL_OFFSET, vex::rotationUnits::deg, power, vex::velocityUnits::pct);
    break;
  //case 1 moves robot left and right
  case 1:
    back_right_motor.startRotateFor(-(degrees*WHEEL_OFFSET), vex::rotationUnits::deg, power, vex::velocityUnits::pct);
    back_left_motor.startRotateFor((degrees*WHEEL_OFFSET), vex::rotationUnits::deg, power, vex::velocityUnits::pct);
    front_right_motor.startRotateFor(degrees*WHEEL_OFFSET, vex::rotationUnits::deg, power, vex::velocityUnits::pct);
    front_left_motor.startRotateFor(-(degrees*WHEEL_OFFSET), vex::rotationUnits::deg, power, vex::velocityUnits::pct);
    break;
  //case 2 handles robot going diagonally with one pair of motors
  case 2:
    back_right_motor.startRotateFor(degrees*WHEEL_OFFSET, vex::rotationUnits::deg, power, vex::velocityUnits::pct);
    front_left_motor.startRotateFor(degrees*WHEEL_OFFSET, vex::rotationUnits::deg, power, vex::velocityUnits::pct);
    break; 
  //case 3 operates same as case 2, but with different pairs of motors
  case 3:
    back_left_motor.startRotateFor(degrees*WHEEL_OFFSET, vex::rotationUnits::deg, power, vex::velocityUnits::pct);
    front_right_motor.startRotateFor(degrees*WHEEL_OFFSET, vex::rotationUnits::deg, power, vex::velocityUnits::pct); 
    break;
  default:
    break;
} // end switch

  
}
//this function will rotate the robor X amount of degrees specified
void rotate(float degrees) {
  //determine necessary measurement to have robot rotate
  float wheelDegrees=TURNING_RATIO*degrees;
  back_right_motor.startRotateFor((wheelDegrees*ROTATION_OFFSET), vex::rotationUnits::deg, power, vex::velocityUnits::pct);
  back_left_motor.startRotateFor(-(wheelDegrees*ROTATION_OFFSET), vex::rotationUnits::deg, power, vex::velocityUnits::pct);
  front_right_motor.startRotateFor((wheelDegrees*ROTATION_OFFSET), vex::rotationUnits::deg, power, vex::velocityUnits::pct);
  front_left_motor.startRotateFor(-(wheelDegrees*ROTATION_OFFSET), vex::rotationUnits::deg, power, vex::velocityUnits::pct);
}


int main(void) {
    vexcodeInit();
    rotate(90);
    while(true) {
        //Get the raw sums of the X and Y joystick axes
        double front_left  = (double)(con.Axis3.position(pct) + con.Axis4.position(pct));
        double back_left   = (double)(con.Axis3.position(pct) - con.Axis4.position(pct));
        double front_right = (double)(con.Axis3.position(pct) - con.Axis4.position(pct));
        double back_right  = (double)(con.Axis3.position(pct) + con.Axis4.position(pct));
        
        //Find the largest possible sum of X and Y
        double max_raw_sum = (double)(abs(con.Axis3.position(pct)) + abs(con.Axis4.position(pct)));
        
        //Find the largest joystick value
        double max_XYstick_value = (double)(std::max(abs(con.Axis3.position(pct)),abs(con.Axis4.position(pct))));
        
        //The largest sum will be scaled down to the largest joystick value, and the others will be
        //scaled by the same amount to preserve directionality
        if (max_raw_sum != 0) {
            front_left  = front_left / max_raw_sum * max_XYstick_value;
            back_left   = back_left / max_raw_sum * max_XYstick_value;
            front_right = front_right / max_raw_sum * max_XYstick_value;
            back_right  = back_right / max_raw_sum * max_XYstick_value;
        }
        
        //Now to consider rotation
        //Naively add the rotational axis
        front_left  = front_left  + con.Axis1.position(pct);
        back_left   = back_left   + con.Axis1.position(pct);
        front_right = front_right - con.Axis1.position(pct);
        back_right  = back_right  - con.Axis1.position(pct);
        
        //What is the largest sum, or is 100 larger?
        max_raw_sum = std::max(std::abs(front_left),std::max(std::abs(back_left),std::max(std::abs(front_right),std::max(std::abs(back_right),100.0))));
        
        //Scale everything down by the factor that makes the largest only 100, if it was over
        front_left  = front_left  / max_raw_sum * 100.0;
        back_left   = back_left   / max_raw_sum * 100.0;
        front_right = front_right / max_raw_sum * 100.0;
        back_right  = back_right  / max_raw_sum * 100.0;
        
        //Write the manipulated values out to the motors
         front_left_motor.spin(fwd,front_left, velocityUnits::pct);
          back_left_motor.spin(fwd,back_left,  velocityUnits::pct);
        front_right_motor.spin(fwd,front_right,velocityUnits::pct);
         back_right_motor.spin(fwd,back_right, velocityUnits::pct);
    }
}
