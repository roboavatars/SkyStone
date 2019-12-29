package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Logger;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.util.Arrays;

@TeleOp
@SuppressWarnings("FieldCanBeLocal")
public class RedTeleop extends LinearOpMode {

    private Robot robot;
    private boolean robotCentric = false;
    private boolean dpadUp = true;
    private boolean dpadDown = true;
    private boolean rightBumper = true;

    @Override
    public void runOpMode() {
        double[] initialPosition = Logger.readPos();
        telemetry.addData("Starting Position", Arrays.toString(initialPosition)); telemetry.update();
        robot = new Robot(this, initialPosition[0], initialPosition[1], initialPosition[2]);
        robot.logger.startLogging();

        waitForStart();
        robot.intake.setControls(1);
        robot.stacker.unClampStone();
        robot.stacker.goHome();

        while (opModeIsActive()) {

            if(gamepad2.x){
                robot.capstoneDeposit.attachCapstone();
            }
            else{
                robot.capstoneDeposit.goHome();
            }

            if(gamepad2.right_trigger>0){
                robot.stacker.basepos += 20;
                robot.stacker.goHome();
            }
            else if(gamepad2.left_trigger>0){
                robot.stacker.basepos -= 20;
                robot.stacker.goHome();
            }

            if(gamepad1.left_bumper){
                robot.deposit();
            }

            if (gamepad1.left_trigger > 0) {
                robot.expelStone();
            }

            if(gamepad1.right_bumper && rightBumper){
                rightBumper = false;
            }
            else if(!rightBumper && !gamepad1.right_bumper){
                robot.swapArmStateTeleop();
                rightBumper = true;
            }
            else if(gamepad2.dpad_up){
                robot.stacker.setDepositControls(0.3, robot.stacker.getArmPosition() - 70);
            }
            else if(gamepad2.dpad_down){
                robot.stacker.setDepositControls(0.3, robot.stacker.getArmPosition() + 70);
            }
            else if(gamepad2.y){
                robot.stacker.setLiftControls(0.3,robot.stacker.getLiftPosition() + 45);
            }
            else if(gamepad2.a){
                robot.stacker.setLiftControls(0.3,robot.stacker.getLiftPosition() - 45);
            }


            if (gamepad1.right_trigger > 0) {
                robot.capstoneDeposit.attachCapstone();
            }

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

            if (gamepad1.dpad_left) robot.grabber.grabFoundation();
            if (gamepad1.dpad_right) robot.grabber.releaseFoundation();

            if (robotCentric) {
                robot.drivetrain.setControls(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            } else {
                robot.drivetrain.setGlobalControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
            }
            robot.update();

            telemetry.addData("Robot Centric", robotCentric);
            telemetry.update();
        }
    }
}