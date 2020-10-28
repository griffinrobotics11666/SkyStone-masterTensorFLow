package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
//@Disabled
public class Continous extends LinearOpMode {
    Servo continousServo;
    double maxPos=1.0;
    double minPos=0;

    public void runOpMode() {
        continousServo = hardwareMap.get(Servo.class, "Test");
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.x){
                continousServo.setPosition(maxPos);
            }
            else if(gamepad1.y){
                continousServo.setPosition(minPos);
            }
            else {
                continousServo.setPosition(0.5);
            }
        }
    }
}