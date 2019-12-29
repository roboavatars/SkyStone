package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "OpenCV2")
public class locatorTest extends LinearOpMode {

    private double[] loc;

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

            telemetry.addData("X", loc[0]);
            telemetry.addData("Y", loc[1]);
            telemetry.addData("Theta", loc[2]);
            telemetry.update();
        }

        locator.setActive(false);
        locator.interrupt();
        telemetry.addData("Status", "Done");
        telemetry.update();
    }
}