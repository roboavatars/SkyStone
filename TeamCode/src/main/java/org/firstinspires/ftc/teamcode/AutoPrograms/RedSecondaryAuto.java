package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@Autonomous
public class RedSecondaryAuto extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {

        // initialize robot
        robot = new Robot(this, 9, 58, 0);
        robot.logger.startLogging();
        robot.grabber.releaseFoundation();
        robot.intake.setControls(0);
        robot.stacker.unClampStone();
        robot.stacker.goHome();

        // after start
        waitForStart();

        ElapsedTime time = new ElapsedTime();

        // robot move loop
        while (opModeIsActive()) {
            robot.update();

            if (time.seconds() > 20){
                robot.drivetrain.setTargetPoint(36,73,0);
            }
        }

        robot.update();
        robot.logger.flush();
        robot.logger.stopLogging();
    }
}