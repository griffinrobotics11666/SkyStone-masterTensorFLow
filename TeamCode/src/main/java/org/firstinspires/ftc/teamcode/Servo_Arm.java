package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.time.chrono.MinguoChronology;

@TeleOp(name="Servo_Arm", group="Concept")
@Disabled
public class Servo_Arm extends LinearOpMode {
    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    double MAX_POS = 9437.0;     // Maximum rotational position
    double MIN_POS = 0.0;     // Minimum rotational position
    boolean debug_switch = false;
    Servo servo;

    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "Robot_Hand");
        telemetry.addData(">", "Press Start to scan Servo.");
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {

            if (gamepad1.a){
                servo.setPosition(MAX_POS);
                /*servo.setPosition(MIN_POS);*/
            }
            else if (gamepad1.b){
                servo.setPosition(MIN_POS);
                /*servo.setPosition(MAX_POS);*/
            }
            else {
                servo.setPosition(0.5);

            }
            telemetry.addData(">", "Press Stop to end test.");
            telemetry.update();

            // Set the servo to the new position and pause;

            if (debug_switch) {
                if (gamepad1.left_bumper) {
                    MAX_POS += INCREMENT;
                    if (MAX_POS >= 1) {
                        MAX_POS = 1;
                    }

                }
                if (gamepad1.left_trigger != 0) ;
                {
                    MIN_POS -= INCREMENT;
                    if (MIN_POS <= 0) {
                        MIN_POS = 0;
                    }


                    // Display the current value
                    telemetry.addData(">", "Press Stop to end test.");
                    telemetry.update();

                    // Set the servo to the new position and pause;
                    idle();
                }

                // Signal done;
                telemetry.addData(">", "Done");
                telemetry.update();
            }
        }
    }
}