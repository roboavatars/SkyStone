package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp(name = "Teleop")
@SuppressWarnings("FieldCanBeLocal")
public class Teleop extends LinearOpMode {
    
    private Robot robot;
    private boolean robotCentric = false;
    private int z = 0;
    private int stackCounter = 0;
    private boolean armOut = false;
    private int centricWait = 0;
    private boolean dpadUp = true;
    private boolean dpadDown = true;

    private boolean rightbumper = true;
    
    @Override
    public void runOpMode() {
        Double[] initialPosition = robot.readPos();
        robot = new Robot(this, initialPosition[0], initialPosition[1], initialPosition[2]);
        //robot.stacker.unClampStone();
        
        waitForStart();
        robot.drivetrain.resetAngle();
        robot.intake.setControls(0);
        robot.stacker.unClampStone();
        robot.stacker.setDepositControls(0.5, 130);
        while (opModeIsActive()) {
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

            if(gamepad1.left_bumper){
                robot.deposit();
            }

            if(gamepad1.right_bumper && rightbumper){
                rightbumper = false;
            }
            else if(!rightbumper && !gamepad1.right_bumper){
                robot.swapArmState();
                rightbumper = true;
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