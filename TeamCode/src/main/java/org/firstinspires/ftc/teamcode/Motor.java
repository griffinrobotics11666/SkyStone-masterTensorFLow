package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Motor Test", group = "Concept")

@Disabled
public class Motor extends LinearOpMode {
    private DcMotor ringToss = null;
    @Override
    public void runOpMode() {
        ringToss = hardwareMap.get(DcMotor.class, "Ring Toss");
        ringToss.setDirection(DcMotorSimple.Direction.FORWARD);
        telemetry.addData(">", "Press Start to scan Motor.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
        if (gamepad1.right_trigger!=0){
            ringToss.setPower(-1);
        }
        if (gamepad1.left_trigger!=0){
            ringToss.setPower(1);
        }
        else{
            ringToss.setPower(0);
        }
        }
    }
}