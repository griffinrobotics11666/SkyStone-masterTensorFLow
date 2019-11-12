/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.HardwareRobot;

/**
 * Demonstrates empty OpMode
 */
@TeleOp(name = "Driver_Control", group = "Concept")
//@Disabled
public class DriverControl extends OpMode {
  HardwareRobot robot = new HardwareRobot();

  private ElapsedTime runtime = new ElapsedTime();

  double openServoL;
  double openServoR;
  double closedServoL;
  double closedServoR;

  //gamepad1
  boolean RB1isPressed;
  boolean LB1isPressed;
  boolean B1isPressed;
  boolean A1isPressed;
  boolean Y1isPressed;
  boolean X1isPressed;
  boolean dPadUp1;
  boolean dPadDown1;
  boolean dPadLeft1;
  boolean dPadRight1;
  boolean R3isPressed1;
  boolean L3isPressed1;

  double RT1;
  double LT1;
  double leftStickY1;
  double leftStickX1;
  double rightStickX1;
  double leftPower;
  double rightPower;

  //gamepad2
  boolean RB2isPressed;
  boolean LB2isPressed;
  boolean B2isPressed;
  boolean A2isPressed;
  boolean Y2isPressed;
  boolean X2isPressed;
  boolean dPadUp2;
  boolean dPadDown2;
  boolean dPadLeft2;
  boolean dPadRight2;
  boolean R3isPressed2;
  boolean L3isPressed2;

  double RT2;
  double LT2;
  double leftStickY2;
  double leftStickX2;
  double rightStickY2;
  double rightStickX2;
  public void gyroMove(double distance, double speed) {
    double WHEEL_DIAMETER = 4;
    double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    double tickperInch = 383.6/WHEEL_CIRCUMFERENCE;
    double deltaSpeed = 0.05;// hardcoded
    double deltaA = 0;
    robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    double currentAngle = robot.angles.firstAngle;

    int newLeftFrontTarget;
    int newRightBackTarget;
    int newRightFrontTarget;
    int newLeftBackTarget;



    //find how many encoder counts the motor is at, then add the distance to it
    newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int)(distance * tickperInch);
    newLeftBackTarget = robot.leftBack.getCurrentPosition() + (int)(distance * tickperInch);
    newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int)(distance * tickperInch);
    newRightBackTarget = robot.rightBack.getCurrentPosition() + (int)(distance * tickperInch);
    //set the target encoder count to the motors
    robot.leftFront.setTargetPosition(newLeftFrontTarget);
    robot.leftBack.setTargetPosition(newLeftBackTarget);
    robot.rightFront.setTargetPosition(newRightFrontTarget);
    robot.rightBack.setTargetPosition(newRightBackTarget);
    //set mode to run to position
    robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //set speed
    robot.leftFront.setPower(Math.abs(speed));
    robot.rightFront.setPower(Math.abs(speed));
    robot.leftBack.setPower(Math.abs(speed));
    robot.rightBack.setPower(Math.abs(speed));
    //While loop is necessary!
    while (robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy() && robot.leftFront.isBusy())
    {
      robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

      if (currentAngle - robot.angles.firstAngle > deltaA){
        telemetry.addData("left", 0);
        robot.leftFront.setPower(speed - deltaSpeed);
        robot.rightFront.setPower(speed + deltaSpeed);
        robot.leftBack.setPower(speed - deltaSpeed);
        robot.rightBack.setPower(speed + deltaSpeed);
      }else if (currentAngle - robot.angles.firstAngle < deltaA){
        telemetry.addData("right", 0);
        robot. leftFront.setPower(speed + deltaSpeed);
        robot. rightFront.setPower(speed - deltaSpeed);
        robot. leftBack.setPower(speed + deltaSpeed);
        robot.rightBack.setPower(speed - deltaSpeed);
      }else {
        telemetry.addData("straight", 0);
        robot.leftFront.setPower(speed);
        robot. rightFront.setPower(speed);
        robot.leftBack.setPower(speed);
        robot.rightBack.setPower(speed);
      }
      telemetry.update();
    }

  }

  public void turn(double angle, double speed){
    robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    double initialAngle = robot.angles.firstAngle;
    double motorPower;
    double minMotorPower = 0.3;
    double powerScaleFactor;
    double targetAngle;
    double currentAngle;
    double deltaAngle;
    double robotAngle = robot.angles.firstAngle;
    double previousAngle = robot.angles.firstAngle;

    targetAngle = initialAngle + angle;

    if (Math.abs(angle) < 8) {
      minMotorPower = .2;
    }

    while (Math.abs(targetAngle - robotAngle) > .5) {
      robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
      currentAngle = robot.angles.firstAngle;

      //update speed dynamically to slow when approaching the target
      powerScaleFactor = Math.sqrt(Math.abs((targetAngle - robotAngle) / angle));
      if (powerScaleFactor > 1) {
        powerScaleFactor = 1;
      }
      motorPower = powerScaleFactor * speed;
      if (motorPower < minMotorPower) {
        motorPower = minMotorPower;
      }

      //determine which direction the robot should turn


      if ((targetAngle - robotAngle) > 0) {
        robot.leftBack.setPower(-motorPower);
        robot.leftFront.setPower(-motorPower);
        robot.rightBack.setPower(motorPower);
        robot.rightFront.setPower(motorPower);
      }
      if ((targetAngle - robotAngle) < 0) {
        robot.leftBack.setPower(motorPower);
        robot.leftFront.setPower(motorPower);
        robot.rightBack.setPower(-motorPower);
        robot.rightFront.setPower(-motorPower);
      }


      //define how the angle is changing and deal with the stupid 180 -> -180 thing
      deltaAngle = currentAngle - previousAngle;
      if (deltaAngle > 180) {
        deltaAngle -= 360;
      } else if (deltaAngle < -180) {
        deltaAngle += 360;
      }

      robotAngle += deltaAngle;
      previousAngle = currentAngle;


      //        double targetAngle, currentangle,multiplier;
//        robot.angles  = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        currentangle = robot.angles.firstAngle;
//        targetAngle = robot.angles.firstAngle + angle;
//
//        while (targetAngle - currentangle != 0){
//            multiplier = Math.abs(targetAngle - currentangle)/targetAngle;
//            currentangle = robot.angles.firstAngle;
//            robot.leftBack.setPower(speed * multiplier);
//            robot.leftFront.setPower(speed * multiplier);
//            robot.rightBack.setPower(-speed * multiplier);
//            robot.rightFront.setPower(-speed * multiplier);
//            telemetry.addData("angle: ",currentangle);
//            telemetry.update();
//        }


    }
  }


  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");

    robot.init(hardwareMap);
  }
  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */


  @Override
  public void init_loop() {
  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    runtime.reset();
  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void loop() {
    RB1isPressed  = gamepad1.right_bumper;
    LB1isPressed  = gamepad1.left_bumper;
    B1isPressed   = gamepad1.b;
    A1isPressed   = gamepad1.a;
    Y1isPressed   = gamepad1.y;
    X1isPressed   = gamepad1.x;
    dPadUp1       = gamepad1.dpad_up;
    dPadDown1     = gamepad1.dpad_down;
    dPadLeft1     = gamepad1.dpad_left;
    dPadRight1    = gamepad1.dpad_right;
    R3isPressed1  = gamepad1.right_stick_button;
    L3isPressed1  = gamepad1.left_stick_button;

    RT1            = gamepad1.right_trigger;
    LT1            = gamepad1.left_trigger;
    leftStickY1    = gamepad1.left_stick_y;
    leftStickX1    = gamepad1.left_stick_x;
    rightStickX1   = gamepad1.right_stick_x;



    //gamepad2
    RB2isPressed = gamepad2.right_bumper;
    LB2isPressed  = gamepad2.left_bumper;
    B2isPressed   = gamepad2.b;
    A2isPressed   = gamepad2.a;
    Y2isPressed   = gamepad2.y;
    X2isPressed   = gamepad2.x;
    dPadUp2       = gamepad2.dpad_up;
    dPadDown2     = gamepad2.dpad_down;
    dPadLeft2     = gamepad2.dpad_left;
    dPadRight2    = gamepad2.dpad_right;
    R3isPressed2  = gamepad2.right_stick_button;
    L3isPressed2  = gamepad2.left_stick_button;

    RT2            = gamepad2.right_trigger;
    LT2            = gamepad2.left_trigger;
    leftStickY2    = gamepad2.left_stick_y;
    leftStickX2    = gamepad2.left_stick_x;
    rightStickY2   = gamepad2.right_stick_y;
    rightStickX2   = gamepad2.right_stick_x;

    telemetry.addData("Status", "Run Time: " + runtime.toString());

    //gamepad1
    RB1isPressed  = gamepad1.right_bumper;
    LB1isPressed  = gamepad1.left_bumper;
    B1isPressed   = gamepad1.b;
    A1isPressed   = gamepad1.a;
    Y1isPressed   = gamepad1.y;
    X1isPressed   = gamepad1.x;
    dPadUp1       = gamepad1.dpad_up;
    dPadDown1     = gamepad1.dpad_down;
    dPadLeft1     = gamepad1.dpad_left;
    dPadRight1    = gamepad1.dpad_right;
    R3isPressed1  = gamepad1.right_stick_button;
    L3isPressed1  = gamepad1.left_stick_button;

    RT1            = gamepad1.right_trigger;
    LT1            = gamepad1.left_trigger;
    leftStickY1    = gamepad1.left_stick_y;
    leftStickX1    = gamepad1.left_stick_x;
    rightStickX1   = gamepad1.right_stick_x;

    //gamepad2
    RB2isPressed  = gamepad2.right_bumper;
    LB2isPressed  = gamepad2.left_bumper;
    B2isPressed   = gamepad2.b;
    A2isPressed   = gamepad2.a;
    Y2isPressed   = gamepad2.y;
    X2isPressed   = gamepad2.x;
    dPadUp2       = gamepad2.dpad_up;
    dPadDown2     = gamepad2.dpad_down;
    dPadLeft2     = gamepad2.dpad_left;
    dPadRight2    = gamepad2.dpad_right;
    R3isPressed2  = gamepad2.right_stick_button;
    L3isPressed2  = gamepad2.left_stick_button;

    RT2            = gamepad2.right_trigger;
    LT2            = gamepad2.left_trigger;
    leftStickY2    = gamepad2.left_stick_y;
    leftStickX2    = gamepad2.left_stick_x;
    rightStickY2   = gamepad2.right_stick_y;
    rightStickX2   = gamepad2.right_stick_x;

    leftPower = Range.clip(-leftStickY1 + leftStickX1, -1.0, 1.0);
    rightPower = Range.clip(-leftStickY1 - leftStickX1, -1.0, 1.0);

    if(leftPower != 0 || rightPower != 0) {
      robot.leftFront.setPower(leftPower);
      robot.rightFront.setPower(rightPower);
      robot.leftBack.setPower(leftPower);
      robot.rightBack.setPower(rightPower);
    }

    else if (rightStickX1 != 0){
      robot.leftFront.setPower(rightStickX1);
      robot.rightFront.setPower(-rightStickX1);
      robot.leftBack.setPower(-rightStickX1);
      robot.rightBack.setPower(rightStickX1);
    }
    else{
      robot.leftFront.setPower(0);
      robot.rightFront.setPower(0);
      robot.leftBack.setPower(0);
      robot.rightBack.setPower(0);
    }
    if(A1isPressed){
      turn(90,1);
    }
    if(A2isPressed){
      if(Math.abs(robot.rightClaw.getPosition() - robot.rightclawclose) < .05) {
        while (Math.abs(robot.horizontalServo.getPosition() - robot.horizontalClawGrab) > .001 && Math.abs(robot.verticalServo.getPosition() - robot.verticalClawGrab) > .001) {
          robot.verticalServo.setPosition(robot.verticalClawGrab);
          robot.horizontalServo.setPosition(robot.horizontalClawGrab);
        }
        robot.leftClaw.setPosition(robot.leftclawclose);
        robot.rightClaw.setPosition(robot.rightclawclose);
      }
      else{
        while (Math.abs(robot.horizontalServo.getPosition() - robot.horizontalClawPlace) > .001 && Math.abs(robot.verticalServo.getPosition() - robot.verticalClawPlace) > .001) {
          robot.verticalServo.setPosition(robot.verticalClawPlace);
          robot.horizontalServo.setPosition(robot.horizontalClawPlace);
        }
        robot.leftClaw.setPosition(robot.leftclawopen);
        robot.rightClaw.setPosition(robot.rightclawopen);
      }
    }
    //if(A1isPressed) {
      //robot.rightWheelServo.setPosition(1);
      //robot.leftWheelServo.setPosition(1);
    //}
  }


}
