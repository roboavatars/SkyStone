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
    
    @Override
    public void runOpMode() {
        robot = new Robot(this, 0, 0, 0);
        //robot.stacker.unClampStone();
        
        waitForStart();
        robot.drivetrain.resetAngle();
        robot.intake.setControls(0);
        robot.stacker.unClampStone();
        robot.stacker.setDepositControls(0.5, 130);
        while (opModeIsActive()) {
            
            z++;
            if (z % 80 == 0 && stackCounter == 0) {
                double distance = robot.stoneSensor.getDistance(DistanceUnit.INCH);
                if (distance < 6) {
                    robot.stacker.setDepositControls(0.5, 50);
                    robot.stacker.clampStone();
                    robot.intake.setControls(0);
                } else {
                    robot.intake.setControls(1);
                }
            }
            if (gamepad1.left_bumper && armOut) {
                robot.stacker.unClampStone();
                robot.stacker.setDepositControls(0.2, (int) robot.stacker.getArmPosition() + 70);
            }
            
            if (gamepad1.right_bumper && stackCounter == 0 && !armOut) {
                robot.stacker.deposit();
                armOut = true;
                stackCounter = 100;
            } else if (gamepad1.right_bumper && stackCounter == 0 && armOut) {
                robot.stacker.setLiftControls(0.5, 0);
                robot.stacker.setDepositControls(0.5, 130);
                robot.stacker.nextLevel();
                stackCounter = 100;
                armOut = false;
            } else if (stackCounter > 0) {
                stackCounter--;
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
            
            if (gamepad1.x && centricWait == 0) {
                robotCentric = !robotCentric;
                centricWait = 100;
            } else if (centricWait > 0) centricWait--;
            
            if (gamepad1.left_trigger > 0) {
                robot.stacker.clampStone();
            }
            if (gamepad1.right_trigger > 0) {
                robot.stacker.unClampStone();
                robot.stacker.setDepositControls(0.5, 130);
                robot.intake.setControls(-1);
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