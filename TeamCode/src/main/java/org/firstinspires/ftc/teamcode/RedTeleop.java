package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Arrays;

@TeleOp
public class RedTeleop extends LinearOpMode {

    private Robot robot;
    private boolean robotCentric = true;
    private boolean dpadUp = true;
    private boolean dpadDown = true;
    private boolean rightBumper = true;
    private boolean leftBumper = true;
    private boolean a = true;

    @Override
    public void runOpMode() {
        double[] initialPosition = Logger.readPos();
        telemetry.addData("Starting Position", Arrays.toString(initialPosition)); telemetry.update();
        robot = new Robot(this, initialPosition[0], initialPosition[1], initialPosition[2]);
        robot.logger.startLogging();

        waitForStart();
        robot.stacker.unClampStone();
        robot.stacker.goHome();
        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper && rightBumper) {
                rightBumper = false;
            } else if (!rightBumper && !gamepad1.right_bumper) {
                robot.deposit();
                rightBumper = true;
            }

            if (gamepad1.left_bumper && leftBumper) {
                leftBumper = false;
            } else if (!leftBumper && !gamepad1.left_bumper) {
                robot.letGo = true;
                leftBumper = true;
            }



            if (gamepad1.b) {robot.intake.setControls(-1);}

            if (gamepad1.a && a) {
                a = false;
            } else if (!gamepad1.a && !a) {
                a = true;
                if (robot.intake.intakeOn()) {robot.intake.setControls(0);}
                else {robot.intake.setControls(0.6);}
            }

            if (gamepad1.dpad_left) robot.grabber.grabFoundation();
            if (gamepad1.dpad_right) robot.grabber.releaseFoundation();

            if (gamepad1.dpad_up && dpadUp) {
                dpadUp = false;
            } else if (!dpadUp && !gamepad1.dpad_up) {
                robot.stacker.nextLevel();
                dpadUp = true;
            }

            if (gamepad1.dpad_down && dpadDown) {
                dpadDown = false;
            } else if (!dpadDown && !gamepad1.dpad_down) {
                robot.stacker.lastLevel();
                dpadDown = true;
            }

            if (robotCentric) {
                robot.drivetrain.setControls(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            } else {
                robot.drivetrain.setGlobalControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }

            double prev = time.milliseconds();
            robot.update();
            double now = time.milliseconds();
            telemetry.addData("loop time", now-prev);
            telemetry.addData("arm ticks", robot.stacker.getArmPosition());
            telemetry.addData("slide ticks", robot.stacker.getLiftPosition());
            telemetry.update();
        }
    }
}