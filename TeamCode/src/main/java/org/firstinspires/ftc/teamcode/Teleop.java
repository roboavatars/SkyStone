package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.PositionLogger;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

import java.io.File;

@TeleOp(name = "Teleop")
@SuppressWarnings("FieldCanBeLocal")
public class Teleop extends LinearOpMode {
    
    private Robot robot;
    private boolean robotCentric = false;
    private int centricWait = 0;
    private boolean dpadUp = true;
    private boolean dpadDown = true;
    private boolean rightBumper = true;
    
    @Override
    public void runOpMode() {
        double[] initialPosition = PositionLogger.readPos();
        robot = new Robot(this, initialPosition[0], initialPosition[1], initialPosition[2]);
        robot.logger.startLogging();
        
        waitForStart();
        robot.drivetrain.resetAngle();
        robot.intake.setControls(1);
        robot.stacker.unClampStone();
        robot.stacker.goHome();

        while (opModeIsActive()) {

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
                robot.swapArmState();
                rightBumper = true;
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

        //Robot.deletePosFile();
    }
}