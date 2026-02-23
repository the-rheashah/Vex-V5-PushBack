/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       rheao                                                     */
/*    Fixed:        PID + Auton Stability                                    */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <cmath>

using namespace vex;

// Competition instance
competition Competition;

// -------------------- MOTORS --------------------
motor RightMotorBack {PORT17};
motor RightMotorFront  {PORT10};
motor LeftMotorBack{PORT5, true}; 
motor LeftMotorFront {PORT15, true};

motor_group LeftMotors  {LeftMotorBack, LeftMotorFront};
motor_group RightMotors {RightMotorFront, RightMotorBack};

drivetrain Drive {LeftMotors, RightMotors};

motor IntakeMotor {PORT1};
motor SpecialButoon {PORT18};
motor MatchLoader {PORT14};

controller Controller(primary);

int IntakeDirection = 0;
int SpecialButoonDirection = 0;

// -------------------- ROBOT SPECS --------------------
const double wheelDiameter = 3.25;
const double trackWidth = 12.5;   // Measure center-to-center


// -------------------- PID VARIABLES --------------------
double kP = 0.42;
double kD = 0.04;

double desiredValue = 0;
double error = 0;
double lastError = 0;
double derivative = 0;

bool enableDrivePID = false;
bool isTurning = false;

// -------------------- PID RESET --------------------
void resetPID() {
  error = 0;
  lastError = 0;
  derivative = 0;
}

// -------------------- PID TASK --------------------
int drivePIDTask() {
  while (true) {
    if (enableDrivePID) {

      double currentPos;
      if (isTurning) {
        currentPos = (LeftMotors.position(degrees)
                    - RightMotors.position(degrees)) / 2.0;
      } else {
        currentPos = (LeftMotors.position(degrees)
                    + RightMotors.position(degrees)) / 2.0;
      }

      error = desiredValue - currentPos;
      derivative = (error - lastError) / 0.02;

      double motorPower = (error * kP) + (derivative * kD);
      // Minimum power (prevents stalling)
      if (fabs(error) > 15) {
        if (motorPower > 0 && motorPower < 6) motorPower = 6;
        if (motorPower < 0 && motorPower > -6) motorPower = -6;
      }

      // Cap max power (VEX-safe)
      if (motorPower > 80) motorPower = 80;
      if (motorPower < -80) motorPower = -80;


      if (isTurning) {
        LeftMotors.spin(forward, motorPower, percent);
        RightMotors.spin(reverse, motorPower, percent);
      } else {
        LeftMotors.spin(forward, motorPower, percent);
        RightMotors.spin(forward, motorPower, percent);
      }

      lastError = error;
    }
    wait(20, msec);
  }
  return 0;
}

//--------------------- Pre Auton --------------------
void pre_auton() {
  LeftMotors.setStopping(coast);
  RightMotors.setStopping(coast);
  IntakeMotor.setStopping(coast);
  IntakeMotor.setVelocity(400, percent);
  RightMotors.setVelocity(300, percent);
  LeftMotors.setVelocity(300, percent);
  MatchLoader.setVelocity(80, percent);
  SpecialButoon.setVelocity(300,percent);
  SpecialButoon.setStopping(hold);

  vex::task t;
  t = vex::task(drivePIDTask);
}


// -------------------- AUTON COMMANDS --------------------
void driveInches(double inches) {
  resetPID();
  isTurning = false;
   enableDrivePID = true;
 
  double targetDegrees =
    (inches * 360.0) / (wheelDiameter * M_PI);

  LeftMotors.resetPosition();
  RightMotors.resetPosition();

  desiredValue = targetDegrees;
  //enableDrivePID = true;

  int settleTime = 0;
  while (settleTime < 10) {
    if (fabs(error) < 5) settleTime++;
    else settleTime = 0;
    wait(20, msec);
  }

  enableDrivePID = false;
  LeftMotors.stop(hold);
  RightMotors.stop(hold);
}

void turnDegrees(double degrees) {
  resetPID();
  isTurning = true;
  enableDrivePID = true;
  double robotCircumference = trackWidth * M_PI;
  double wheelCircumference = wheelDiameter * M_PI;
  double ratio = robotCircumference / wheelCircumference;

  double targetWheelDegrees = degrees * ratio/2.0; 

  LeftMotors.resetPosition();
  RightMotors.resetPosition();

  desiredValue = targetWheelDegrees;
  

  int settleTime = 0;
  while (settleTime < 10) {
    if (fabs(error) < 5) settleTime++;
    else settleTime = 0;
    wait(20, msec);
  }

  enableDrivePID = false;
  LeftMotors.stop(hold);
  RightMotors.stop(hold);
}
// -------------------- OTHER FUNCTIONS --------------------

void IntakeOn() {
  IntakeMotor.spin(forward, 100, percent);
}

void IntakeReverse() {
  if (IntakeDirection == 1) { 
    IntakeMotor.stop(); 
    IntakeDirection = 0; 
  } else { 
    IntakeMotor.spin(reverse); IntakeDirection = -1; 
  } 
} 


void IntakeOff() {
  IntakeMotor.stop();
}

/*void specialbuttonforward() {
  if (SpecialButoonDirection == -1) { 
  SpecialButoon.stop(); 
  SpecialButoonDirection = 0; 
  } else { 
    SpecialButoon.spin(forwreverserd); SpecialButoonDirection = 1;
   }
  }*/
// -------------------- AUTON --------------------
void autonomous(void) {
  //Code built for compition on 2/21 and its working for right side. 
  
  IntakeMotor.setVelocity(300, percent);
  //AngleChanger.setStopping(hold);
  LeftMotors.setVelocity(70, percent);
  RightMotors.setVelocity(70, percent);
  LeftMotors.setStopping(coast);
  RightMotors.setStopping(coast); 
  driveInches(-18);
  turnDegrees(170);
  IntakeOn();
  driveInches(-18);
  IntakeOff();
  turnDegrees(90);
  driveInches(-27);
  turnDegrees(55);
  driveInches(21);
  IntakeOn();
  SpecialButoon.spin(reverse, 300, percent);

}

// -------------------- DRIVER BUTTONS --------------------
void ButtonR1Pressed() { 
  if (IntakeDirection == 1) { 
    IntakeMotor.stop(); 
    IntakeDirection = 0; 
  } else { 
    IntakeMotor.spin(reverse); IntakeDirection = -1; 
  } 
} 

void ButtonR2Pressed() { 
  if (IntakeDirection == -1) { 
    IntakeMotor.stop(); 
    IntakeDirection = 0; 
  } else { 
    IntakeMotor.spin(forward); IntakeDirection = 1;
   }
  } 
  
  void ButtonL2Pressed() { 
   if (SpecialButoonDirection == -1) { 
    SpecialButoon.stop(); 
    SpecialButoonDirection = 0; 
  } else { 
    SpecialButoon.spin(reverse); SpecialButoonDirection = -1;
   }
  }
  


  void ButtonL1Pressed() {
   if (SpecialButoonDirection == -1) { 
    SpecialButoon.stop(); 
    SpecialButoonDirection = 0; 
  } else { 
    SpecialButoon.spin(reverse); SpecialButoonDirection = -1;
   }
  }


void ButtondownPressed() {
  MatchLoader.spinToPosition(320, degrees);
}

void ButtonupPressed() {
  MatchLoader.spinToPosition(0, degrees);
}

// -------------------- DRIVER CONTROL --------------------
void usercontrol(void) {

  enableDrivePID = false;

  while (true) {

    Drive.arcade(
      Controller.Axis3.value(),
      Controller.Axis1.value()
    );

    wait(20, msec);
  }
}

// -------------------- MAIN --------------------
int main() {    

  LeftMotors.setStopping(hold);
  RightMotors.setStopping(hold);
  pre_auton();

  //Competition.pre_auton(pre_auton);
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  Controller.ButtonR1.pressed(ButtonR1Pressed);
  Controller.ButtonR2.pressed(ButtonR2Pressed);
  Controller.ButtonL1.pressed(ButtonL1Pressed);
  Controller.ButtonL2.pressed(ButtonL2Pressed);
  Controller.ButtonUp.pressed(ButtonupPressed);
  Controller.ButtonDown.pressed(ButtondownPressed);

  vex::task x;
  x = vex::task(drivePIDTask);

  while (true) {
    wait(100, msec);
  }
}
