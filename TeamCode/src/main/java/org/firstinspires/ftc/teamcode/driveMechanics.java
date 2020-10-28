package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp()
@Disabled
public class driveMechanics extends LinearOpMode {
    double drive;
    double strafe;
    //right stick
    double turn;
    double rearLeftPower;
    double rearRightPower;
    double frontLeftPower;
    double frontRightPower;
    private DcMotor rearLeftDrive = null;
    private DcMotor rearRightDrive = null;
    private DcMotor frontLeftDrive=null;
    private DcMotor frontRightDrive=null;
//    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        rearLeftDrive = hardwareMap.get(DcMotor.class, "Left");
        rearRightDrive = hardwareMap.get(DcMotor.class, "Right");
        rearLeftDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        waitForStart();


        // Scan servo till stop pressed.
        while (opModeIsActive()) {
            turn=gamepad1.right_stick_x;
            drive=gamepad1.left_stick_y;
            strafe=gamepad1.left_stick_x;
            rearLeftPower=drive - strafe + turn;
            rearRightPower=drive + strafe - turn;
            rearLeftPower=Range.clip(drive - strafe + turn, -1, 1);
            rearRightPower=Range.clip(drive + strafe - turn, -1, 1);
            frontLeftPower=Range.clip(drive + strafe + turn, -1, 1);
            frontRightPower=Range.clip(drive - strafe - turn, -1, 1);
 //           telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", rearLeftPower, rearRightPower);

            telemetry.update();
            // Send calculated power to rear wheels
            rearLeftDrive.setPower(rearLeftPower);
            rearRightDrive.setPower(rearRightPower);
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
        }
    }
}