package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;
    @TeleOp(name="debugMode", group="Auto")
//    @Disabled
 public class debug extends OpMode {
        double movement = .0005;
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
        boolean isReverse;

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
        HardwareRobot robot = new HardwareRobot();
        boolean changed = false, on = false; //Outside of loop()

        private ElapsedTime runtime = new ElapsedTime();

        @Override
        public void init() {
            telemetry.addData("Status", "Initialized");
            robot.init(hardwareMap);
            robot.leftWheel.setPower(1);
            robot.rightWheel.setPower(1);
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
            NormalizedRGBA colors = robot.colorSensor.getNormalizedColors();


            if(true) {
                RB1isPressed = gamepad1.right_bumper;
                LB1isPressed = gamepad1.left_bumper;//
                B1isPressed = gamepad1.b;//
                A1isPressed = gamepad1.a;//
                Y1isPressed = gamepad1.y;//
                X1isPressed = gamepad1.x;//
                dPadUp1 = gamepad1.dpad_up;
                dPadDown1 = gamepad1.dpad_down;//
                dPadLeft1 = gamepad1.dpad_left;
                dPadRight1 = gamepad1.dpad_right;
                R3isPressed1 = gamepad1.right_stick_button;
                L3isPressed1 = gamepad1.left_stick_button;

                RT1 = gamepad1.right_trigger;
                LT1 = gamepad1.left_trigger;
                leftStickY1 = gamepad1.left_stick_y;
                leftStickX1 = gamepad1.left_stick_x;
                rightStickX1 = gamepad1.right_stick_x;


                //gamepad2
                RB2isPressed = gamepad2.right_bumper;
                LB2isPressed = gamepad2.left_bumper;
                B2isPressed = gamepad2.b;//
                A2isPressed = gamepad2.a;//
                Y2isPressed = gamepad2.y;//
                X2isPressed = gamepad2.x;//
                dPadUp2 = gamepad2.dpad_up;
                dPadDown2 = gamepad2.dpad_down;
                dPadLeft2 = gamepad2.dpad_left;
                dPadRight2 = gamepad2.dpad_right;
                R3isPressed2 = gamepad2.right_stick_button;
                L3isPressed2 = gamepad2.left_stick_button;

                RT2 = gamepad2.right_trigger;//
                LT2 = gamepad2.left_trigger;//
                leftStickY2 = gamepad2.left_stick_y;
                leftStickX2 = gamepad2.left_stick_x;
                rightStickY2 = gamepad2.right_stick_y;
                rightStickX2 = gamepad2.right_stick_x;
            }
            if(LB1isPressed) {
                if (A1isPressed) {
                    robot.armMotor.setPower(.5);
                }
                else {
                    robot.armMotor.setPower(0);
                }
                if(B1isPressed) {
                    robot.armMotor.setPower(-.5);
                }
                else {
                    robot.armMotor.setPower(0);
                }
            }else {
                if (A1isPressed) {
                    robot.leftWheelServo.setPosition(robot.leftWheelServo.getPosition() + movement);
                }
                if (B1isPressed) {
                    robot.leftWheelServo.setPosition(robot.leftWheelServo.getPosition() - movement);
                }
                if (Y1isPressed) {
                    robot.rightWheelServo.setPosition(robot.rightWheelServo.getPosition() + movement);
                }
                if (X1isPressed) {
                    robot.rightWheelServo.setPosition(robot.rightWheelServo.getPosition() - movement);
                }
            }
            if(!RB2isPressed) {
                if (A2isPressed) {
                    robot.rightClaw.setPosition(robot.rightClaw.getPosition() - movement);
                }
                if (B2isPressed) {
                    robot.rightClaw.setPosition(robot.rightClaw.getPosition() + movement);
                }
                if (X2isPressed) {
                    robot.leftClaw.setPosition(robot.leftClaw.getPosition() - movement);
                }
                if (Y2isPressed) {
                    robot.leftClaw.setPosition(robot.leftClaw.getPosition() + movement);
                }
            }else{
//                if (A2isPressed) {
//                    robot.horizontalServo.setPosition(robot.horizontalServo.getPosition() - movement);
//                }
//                if (B2isPressed) {
//                    robot.horizontalServo.setPosition(robot.horizontalServo.getPosition() + movement);
//                }
                if (X2isPressed) {
                    robot.verticalServo.setPosition(robot.verticalServo.getPosition() - movement);
                }
                if (Y2isPressed) {
                    robot.verticalServo.setPosition(robot.verticalServo.getPosition() + movement);
                }
            }
            if(dPadDown1){
               if(isReverse){
                   robot.rightWheel.setDirection(DcMotor.Direction.REVERSE);
                   robot.leftWheel.setDirection(DcMotor.Direction.FORWARD);
               }else{
                   robot.leftWheel.setDirection(DcMotor.Direction.REVERSE);
                   robot.rightWheel.setDirection(DcMotor.Direction.FORWARD);
               }
                isReverse = !isReverse;
            }
            if(RT2 != 0){
                robot.lift.setPower(RT2);
            }else if(LT2 != 0){
                robot.lift.setPower(-LT2);
            }else{
                robot.lift.setPower(0);
            }
             //Outside of loop()
            //Amazing toggle
            if(dPadDown1 && !changed) {
                robot.rightWheel.setPower(on ? 1 : -1);
                robot.leftWheel.setPower(on ? 1 : -1);
                on = !on;
                changed = true;
            } else if(!gamepad1.a) changed = false;

            if(dPadDown2){
                robot.rightGrabServo.setPosition(robot.rightGrabServo.getPosition() + movement);
            }else if (dPadRight2){
                robot.rightGrabServo.setPosition(robot.rightGrabServo.getPosition() - movement);
            }
            if(dPadLeft2){
                robot.leftGrabServo.setPosition(robot.leftGrabServo.getPosition() + movement);
            }
            else if (dPadUp2){
                robot.leftGrabServo.setPosition(robot.leftGrabServo.getPosition() - movement);
            }





            //right servo .288 (open) 1(close)(TODO NEED NEW POSITIONS)
            //left servo .68 (open) 0 (close) (TODO NEED NEW POSITIONS)

            int color = colors.toColor();
            telemetry.addLine("raw Android color: ")
                    .addData("a", "%02x", Color.alpha(color))
                    .addData("r", "%02x", Color.red(color))
                    .addData("g", "%02x", Color.green(color))
                    .addData("b", "%02x", Color.blue(color));

            telemetry.addData("Right motor direction",robot.rightWheel.getDirection());
            telemetry.addData("Left motor direction",robot.leftWheel.getDirection());
            telemetry.addData("right grab",robot.rightGrabServo.getPosition());
            telemetry.addData("left grab",robot.leftGrabServo.getPosition());

            telemetry.addData("Left servo",robot.leftWheelServo.getPosition());

            telemetry.addData("Right servo",robot.rightWheelServo.getPosition());

            telemetry.addData("left claw",robot.leftClaw.getPosition());
            telemetry.addData("left claw",robot.rightClaw.getPosition());
           // telemetry.addData("horizontal servo (x)",robot.horizontalServo.getPosition());
            telemetry.addData("vertical servo (y)",robot.verticalServo.getPosition());


            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }




