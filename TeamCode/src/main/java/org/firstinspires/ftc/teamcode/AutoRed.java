package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Autonomous Red", group="Auto")
@Disabled
public class AutoRed extends OpMode {
    HardwareRobot robot = new HardwareRobot();

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        robot.init(hardwareMap);

    }

    /*
     * Code to run when the op mode  is first enabled goes here
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
        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

}
