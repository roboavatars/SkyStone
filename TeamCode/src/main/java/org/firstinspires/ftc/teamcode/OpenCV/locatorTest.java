package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Autonomous(name = "OpenCV2")
public class locatorTest extends LinearOpMode {

    private Robot robot;
    private double[] loc;
    private double time;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(this, 0, 0, 0, false);
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

            double x = loc[0];
            double y = loc[1];
            double r = Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
            double theta = robot.drivetrain.currentheading;
            robot.drivetrain.setTargetPoint(robot.drivetrain.x + r*Math.cos(theta),robot.drivetrain.y + r*Math.sin(theta),loc[2] + robot.drivetrain.currentheading);


            telemetry.addData("Local X", x);
            telemetry.addData("Local Y", y);
            telemetry.addData("Local Theta", loc[2]);
            telemetry.addData("FPS", 1000 / time);
            telemetry.addData("Global X", robot.drivetrain.x + r*Math.cos(theta));
            telemetry.addData("Global Y", robot.drivetrain.y + r*Math.sin(theta));
            telemetry.addData("Global Theta", loc[2] + robot.drivetrain.currentheading);
            telemetry.update();
            robot.update();
        }

        locator.setActive(false);
        locator.interrupt();
        telemetry.addData("Status", "Done");
        telemetry.update();
    }
}