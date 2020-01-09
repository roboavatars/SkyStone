package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "OpenCV2")
public class locatorTest extends LinearOpMode {

    private double[] loc;
    private double time;
    private double n = 1;
    private double m = 1;
    private double o = 0;

    @Override
    public void runOpMode() {
        stoneLocator2 locator = new stoneLocator2(this);
        locator.initializeCamera();
        locator.start();
        locator.setActive(true);
        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            loc = locator.getLocation();
            time = locator.getTime();

            telemetry.addData("Raw X", loc[0]);
            telemetry.addData("Raw Y", loc[1]);
            telemetry.addData("Raw Theta", loc[2]);
            telemetry.addData("FPS", 1000 / time);

            telemetry.addData("X", (240 - loc[0]) * n * m);
            telemetry.addData("Y", (180 - loc[1]) * n + o);

            n += gamepad1.left_stick_y;
            m += gamepad1.right_stick_y;
            o += gamepad1.right_trigger;
            o -= gamepad1.left_trigger;

            telemetry.update();
        }

        locator.setActive(false);
        locator.interrupt();
        telemetry.addData("Status", "Done");
        telemetry.update();
    }
}