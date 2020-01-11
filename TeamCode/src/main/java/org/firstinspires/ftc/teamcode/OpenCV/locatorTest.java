package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Autonomous(name = "OpenCV2")
public class locatorTest extends LinearOpMode {

    private Robot robot;
    private double[] loc;
    private double time;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 0, 0, 0);
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

            telemetry.addData("Local X", loc[0]);
            telemetry.addData("Local Y", loc[1]);
            telemetry.addData("Local Theta", loc[2]);
            telemetry.addData("FPS", 1000 / time);
            telemetry.addData("Global X", loc[0] + robot.drivetrain.x);
            telemetry.addData("Global Y", loc[1] + robot.drivetrain.y);
            telemetry.addData("Global Theta", loc[2] + robot.drivetrain.currentheading);
            telemetry.update();
        }

        locator.setActive(false);
        locator.interrupt();
        telemetry.addData("Status", "Done");
        telemetry.update();
    }
}