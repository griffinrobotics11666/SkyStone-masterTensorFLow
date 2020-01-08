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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class   HardwareRobot
{
    BNO055IMU imu;

    // State used for updating telemetry
    Orientation angles;
    Acceleration gravity;
    /* Public OpMode members. */
    //Motors that move the robot
    public DcMotor leftFront   = null;
    public DcMotor rightFront  = null;
    public DcMotor leftBack    = null;
    public DcMotor rightBack   = null;
    public DcMotor armMotor    = null;
    public DcMotor lift        = null;
    //Motors that move the wheels grabbing stones
    public DcMotor rightWheel  = null;
    public DcMotor leftWheel   = null;
    //Servos of the claw
    public Servo  leftClaw     = null;
    public Servo  rightClaw    = null;
    //servos that twist the claw
    public Servo verticalServo   = null;
    //public Servo horizontalServo = null;
//    //Servos that move the wheels grabbing stones
    public Servo rightWheelServo = null;
    public Servo leftWheelServo  = null;
    //grab servos on the front
    public Servo leftGrabServo = null;
    public Servo rightGrabServo = null;
    public DigitalChannel magSwitch = null;

    public ColorSensor colorSensor;




    public static final double MID_SERVO       =  .5;
    public double rightclawopen = .1255;
    public double rightclawclose = 0.06;
    public double leftclawopen = .8983;
    public double leftclawclose = .96;
//    public double horizontalClawGrab = .68;
//    public double horizontalClawPlace = .12;
    public double verticalClawGrab = .68;
    public double verticalClawPlace = .03;
    public double leftgrabopen = .23;
    public double leftgrabclosed = .76;
    public double rightgrabopen = .52;
    public double rightgrabclose = .016 ;
    public double rightintakeservostow =.126;
    public double rightintakeservograb = .27;
    public double leftintakeservostow = .92;
    public double leftintakeservograb = .77;
    public double leftintakeopen = .5916;
    public double rightintakeopen = .4305;
    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareRobot(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap){
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront  = hwMap.get(DcMotor.class, "left_front");
        rightFront = hwMap.get(DcMotor.class, "right_front");
        leftBack   = hwMap.get(DcMotor.class, "left_back");
        rightBack  = hwMap.get(DcMotor.class, "right_back");
        armMotor   = hwMap.get(DcMotor.class, "arm_motor");
        rightWheel = hwMap.get(DcMotor.class, "intake_right");
        leftWheel  = hwMap.get(DcMotor.class, "intake_left");
        lift       = hwMap.get(DcMotor.class, "lift_motor");
        colorSensor = hwMap.get(ColorSensor.class, "sensor_color");
        magSwitch = hwMap.get(DigitalChannel.class, "mag_touch");

        leftFront.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightFront.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setDirection(DcMotor.Direction.FORWARD); //TODO CHECK THIS OR DIE
        rightWheel.setDirection(DcMotor.Direction.REVERSE);
        leftWheel.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD); //TODO CHECK THIS OR DIE v.2
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        leftFront.setPower(0);
        rightFront.setPower(0);
        leftBack.setPower(0);
        rightBack.setPower(0);
        armMotor.setPower(0);
        lift.setPower(0);
        //        rightWheel.setPower(0);
//        leftWheel.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER );

        // Define and initialize ALL installed servos.
        leftClaw        = hwMap.get(Servo.class, "left_hand");
        rightClaw       = hwMap.get(Servo.class, "right_hand");
        verticalServo   = hwMap.get(Servo.class, "y_servo");
        //horizontalServo = hwMap.get(Servo.class, "x_servo");
        rightWheelServo = hwMap.get(Servo.class, "right_wheel_servo");
        leftWheelServo  = hwMap.get(Servo.class, "left_wheel_servo");
        leftGrabServo   = hwMap.get(Servo.class, "left_grab");
        rightGrabServo  = hwMap.get(Servo.class, "right_grab");
        leftClaw.setPosition(leftclawopen);
        rightClaw.setPosition(rightclawopen);
        verticalServo.setPosition(verticalClawPlace);
        rightGrabServo.setPosition(rightgrabopen);
        leftGrabServo.setPosition(leftgrabopen);
        //horizontalServo.setPosition(horizontalClawGrab);
        leftWheelServo.setPosition(leftintakeservostow);
        rightWheelServo.setPosition(rightintakeservostow);
//        rightClaw.setDirection(Servo.Direction.REVERSE);

        //Limit Switch
        magSwitch.setMode(DigitalChannel.Mode.INPUT);

        //Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu"..
        imu = hwMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }
 }