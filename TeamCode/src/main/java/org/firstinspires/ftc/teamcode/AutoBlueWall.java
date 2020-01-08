package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="Autonomous Blue(Wall)", group="Auto")
//@Disabled
public class AutoBlueWall extends LinearOpMode {
    HardwareRobot robot = new HardwareRobot();
    float hsvValues[] = {0F, 0F, 0F};
    final double SCALE_FACTOR = 255;
    public void stopRobot() {
        wheelSetMode(2);

        robot.leftBack.setPower(0);
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack.setPower(0);

    }
    public void strafeToLine(boolean isLeft, double speed){
        wheelSetMode(2);
        Color.RGBToHSV((int) (robot.colorSensor.red() * SCALE_FACTOR),
                (int) (robot.colorSensor.green() * SCALE_FACTOR),
                (int) (robot.colorSensor.blue() * SCALE_FACTOR),
                hsvValues);
        while(Math.abs(robot.colorSensor.red() - robot.colorSensor.blue()) < 200){
            if(!isLeft) {
                robot.leftFront.setPower(Math.abs(speed));
                robot.rightFront.setPower(-Math.abs(speed));
                robot.leftBack.setPower(-Math.abs(speed));
                robot.rightBack.setPower(Math.abs(speed));
            }else{
                robot.leftFront.setPower(-Math.abs(speed));
                robot.rightFront.setPower(Math.abs(speed));
                robot.leftBack.setPower(Math.abs(speed));
                robot.rightBack.setPower(-Math.abs(speed));
            }
        }
        stopRobot();
    }
    public void strafe(double distance, double speed) {
        int newLeftFrontTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;
        double strafeScale =(10000.0/98.0) * (24.0/27.0) * (24.0/27.0);

        wheelSetMode(1);

        newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int) (distance * strafeScale);
        newLeftBackTarget = robot.leftBack.getCurrentPosition() - (int) (distance * strafeScale);
        newRightFrontTarget = robot.rightFront.getCurrentPosition() - (int) (distance * strafeScale);
        newRightBackTarget = robot.rightBack.getCurrentPosition() + (int) (distance * strafeScale);

        robot.leftFront.setTargetPosition(newLeftFrontTarget);
        robot.leftBack.setTargetPosition(newLeftBackTarget);
        robot.rightFront.setTargetPosition(newRightFrontTarget);
        robot.rightBack.setTargetPosition(newRightBackTarget);

        robot.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFront.setPower(Math.abs(speed));
        robot.rightFront.setPower(Math.abs(speed));
        robot.leftBack.setPower(Math.abs(speed)); //-
        robot.rightBack.setPower(Math.abs(speed)); //-

        while (robot.leftBack.isBusy() && robot.leftFront.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy() && opModeIsActive()){

        }
    }
    public void wheelSetMode(int mode){
        robot.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if(mode == 1){
            robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if(mode == 2){
            robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }


    }

    private ElapsedTime runtime = new ElapsedTime();
    public void gyroMove (double distance, double speed){

        double WHEEL_DIAMETER = 4;
        double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
        double tickperInch = (2 * 383.6) / WHEEL_CIRCUMFERENCE;
        double deltaSpeed = 0.05;// hardcoded
        double deltaA = 0;
        double whithin1ft = tickperInch * 12;
        double powerinft;
        robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = robot.angles.firstAngle;

        int newLeftFrontTarget;
        int newRightBackTarget;
        int newRightFrontTarget;
        int newLeftBackTarget;

        wheelSetMode(1);


        //find how many encoder counts the motor is at, then add the distance to it
        newLeftFrontTarget = robot.leftFront.getCurrentPosition() + (int) (distance * tickperInch);
        newLeftBackTarget = robot.leftBack.getCurrentPosition() + (int) (distance * tickperInch);
        newRightFrontTarget = robot.rightFront.getCurrentPosition() + (int) (distance * tickperInch);
        newRightBackTarget = robot.rightBack.getCurrentPosition() + (int) (distance * tickperInch);
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
        while (robot.leftBack.isBusy() && robot.rightFront.isBusy() && robot.rightBack.isBusy() && robot.leftFront.isBusy() && opModeIsActive()) {
            robot.angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            if( robot.leftBack.getTargetPosition() - robot.leftBack.getCurrentPosition() < whithin1ft){
                powerinft = .5;
            }else{
                powerinft = 1;
            }

            if (currentAngle - robot.angles.firstAngle > deltaA) {
                telemetry.addData("left", 0);
                robot.leftFront.setPower((speed - deltaSpeed) * powerinft);
                robot.rightFront.setPower((speed + deltaSpeed) * powerinft);
                robot.leftBack.setPower((speed - deltaSpeed) * powerinft);
                robot.rightBack.setPower((speed + deltaSpeed) * powerinft);
            } else if (currentAngle - robot.angles.firstAngle < deltaA) {
                telemetry.addData("right", 0);
                robot.leftFront.setPower((speed + deltaSpeed)* powerinft);
                robot.rightFront.setPower((speed - deltaSpeed)*powerinft);
                robot.leftBack.setPower((speed + deltaSpeed)*powerinft);
                robot.rightBack.setPower((speed - deltaSpeed)*powerinft);
            } else {
                telemetry.addData("straight", 0);
                robot.leftFront.setPower((speed));
                robot.rightFront.setPower((speed));
                robot.leftBack.setPower((speed));
                robot.rightBack.setPower((speed));
            }
            telemetry.update();
        }
        stopRobot();

    }

    public void turn( double angle, double speed){
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


        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        targetAngle = initialAngle + angle;

        if (Math.abs(angle) < 8) {
            minMotorPower = .2;
        }


        while (Math.abs(targetAngle - robotAngle) > .5 && opModeIsActive()) {
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
        stopRobot();
    }
    // TODO GET A BUTTON TO RESET THE ENCODER COUNTS ON THE ARM OF THE ROBOT
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);


        waitForStart();
        runtime.reset();

//        strafeToLine(true ,.7);
/*
        gyroMove(24*2 - 18-3,1);
        gyroMove(3,.5);
        gyroMove(3,.3);
        robot.rightGrabServo.setPosition(robot.rightgrabclose);
        robot.leftGrabServo.setPosition(robot.leftgrabclosed);
        sleep(1000);
        gyroMove(-24*2 + 18,1);
        gyroMove(-9,.5);
        robot.rightGrabServo.setPosition(robot.rightgrabopen);
        robot.leftGrabServo.setPosition(robot.leftgrabopen);
        strafe(+18 + 5 ,1);
        gyroMove(12*2 - 5,1);
        strafe(-24,1);
        strafe(24,1);
        gyroMove(-12*3 + 9,1);

*/
        strafe(-9,.5);
        gyroMove(24*2 - 18-3,1);
        gyroMove(3,.5);
        gyroMove(3,.3);
        robot.rightGrabServo.setPosition(robot.rightgrabclose);
        robot.leftGrabServo.setPosition(robot.leftgrabclosed);
        sleep(1000);
        gyroMove(-24*2 + 18,1);
        gyroMove(-9,.5);
        robot.rightGrabServo.setPosition(robot.rightgrabopen);
        robot.leftGrabServo.setPosition(robot.leftgrabopen);
        strafe(48 ,1);
    }
}