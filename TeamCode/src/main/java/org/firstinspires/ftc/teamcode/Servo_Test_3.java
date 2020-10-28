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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import java.util.Random;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left_hand" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Servo Test", group = "Concept")

//@Disabled
public class Servo_Test_3 extends LinearOpMode {

    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
   /* double startPosition50 = 0.38;
    double endPosition50 = 0.28;
    double startPosition25 = 0.38;
    double endPosition25 = 0.48;
    */
    double startContinous = 0.0;
    double endContinous = 1;
    double rotationStart50 = 0;
    double rotationEnd50=0.5;
    double rotationStart25=0;
    double rotationEnd25 = 0.25;
    double drive;
    double turn;
    double leftPower;
    double rightPower;
    boolean debug_switch = true;
    public DcMotor leftDrive = null;
    public DcMotor rightDrive = null;
    // Define class members
    Servo servo; //Continous
    Servo servoRotational50; //Rotation Half
    Servo servoRotational25; //Rotational Quarter


    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        //servo = hardwareMap.get(Servo.class, "Continuous");
        //servoRotational50 = hardwareMap.get(Servo.class, "Half Rotation");
        //servoRotational25 = hardwareMap.get(Servo.class, "Quarter Rotation");
        leftDrive = hardwareMap.get(DcMotor.class, "Left");
        rightDrive = hardwareMap.get(DcMotor.class, "Right");
        leftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            turn = gamepad1.left_stick_x;
            drive = -gamepad1.right_stick_y;
            leftPower = drive + turn;
            rightPower = drive - turn;
            // slew the servo, according to the rampUp (direction) variable.
            /*if (debug_switch) {
                if (gamepad1.a) {
                    servoRotational50.setPosition(rotationEnd50);
                    servoRotational25.setPosition(rotationEnd25);
                }
                if (gamepad1.b) {
                    servoRotational50.setPosition(rotationStart50);
                    servoRotational25.setPosition(rotationStart25);
                }
                if (gamepad1.dpad_down) {
                    servo.setPosition(endContinous);
                }
                else if (gamepad1.dpad_up) {
                    servo.setPosition(startContinous);
                }
                else{
                    servo.setPosition(0.5);
                }
                */
                leftDrive.setPower(Range.clip(leftPower, -1, 1));

                rightDrive.setPower(Range.clip(rightPower, -1, 1));
            }
        }

        /*if (debug_switch) {
            if (gamepad1.left_bumper) {
                position += INCREMENT;
                if (position >= 1) {
                    position = 1;
                }
            }
            if (gamepad1.right_bumper) {
                position_2 -= INCREMENT;
                if (position_2 <= 0) {
                    position_2 = 0;
                }
            }
            if (gamepad1.left_trigger != 0) ;
            {
                position -= INCREMENT;
                if (position <= 0) {
                    position = 0;
                }
            }
            if (gamepad1.right_trigger != 0) {
                position_2 += INCREMENT;
                if (position_2 >= 1) {
                    position_2 = 1;
                }
            }
*/
            // Display the current value
            //telemetry.addData(">", "Press Stop to end test.");
            //telemetry.update();

            // Set the servo to the new position and pause;
            //telemetry.addData("Vertical_Servo", "%5.2", servo.getPosition());
            ;
            //telemetry.addData(">", "Done");
            //telemetry.update();
        //}
    }



/*

    //left stick
    double drive  =  gamepad1.left_stick_y;
    double strafe = -gamepad1.left_stick_x;
    //right stick
    double turn = -gamepad1.right_stick_x;

//calculates power
                    rearLeftPower    = Range.clip(drive - strafe + turn, -1, 1) ;
                    rearRightPower   = Range.clip(drive + strafe - turn, -1, 1) ;

                    frontLeftPower = Range.clip(drive + strafe + turn, -1, 1) ;
                    frontRightPower = Range.clip(drive - strafe - turn, -1, 1) ;

                    // Send calculated power to rear wheels
                    rearLeftDrive.setPower(rearLeftPower);
                    rearRightDrive.setPower(rearRightPower);
                    frontLeftDrive.setPower(frontLeftPower);
                    frontRightDrive.setPower(frontRightPower);

 */