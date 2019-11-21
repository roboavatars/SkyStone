package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp(name="Teleop") @SuppressWarnings("FieldCanBeLocal")
public class Teleop extends LinearOpMode {

    private Robot robot;

    private double intakePower = 0;
    private double armPower = 0;
    private double liftPower = 0;

    private final boolean robotCentric = true;

    @Override
    public void runOpMode() {
        robot = new Robot(this, 0 , 0,0);
        //robot.stacker.unClampStone();
        ElapsedTime xBuffer = new ElapsedTime();

        waitForStart();
        robot.drivetrain.resetAngle();
        robot.intake.setControls(0);

        while (opModeIsActive()){

            if (gamepad1.x && xBuffer.milliseconds()>1000 && intakePower == 0) {
                intakePower = 1;
                xBuffer.reset();
                xBuffer.startTime();
            } else if (gamepad1.x && xBuffer.milliseconds()>1000) {
                intakePower = 0;
                xBuffer.reset();
                xBuffer.startTime();
            }
    
            double distance = robot.stoneSensor.getDistance(DistanceUnit.INCH);
            if (distance < 2) {
                telemetry.addData("stone", "detected");
            } else {
                telemetry.addData("stone", "not detected");
            }
            
            if (gamepad1.dpad_down) robot.grabber.grabFoundation();
            if (gamepad1.dpad_up) robot.grabber.releaseFoundation();

            if (gamepad1.a) robot.stacker.clampStone();
            else if (gamepad1.b) robot.stacker.unClampStone();
            
            armPower = gamepad1.right_trigger*0.25 - gamepad1.left_trigger*0.25;
            liftPower = gamepad2.right_trigger*0.25 - gamepad2.left_trigger*0.25;
    
            if (robotCentric) {robot.drivetrain.setControls(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);}
            else {robot.drivetrain.setGlobalControls(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);}
            robot.update();
            robot.intake.setControls(intakePower);

            telemetry.addData("X", robot.drivetrain.x);
            telemetry.addData("Y", robot.drivetrain.y);
            telemetry.addData("Theta", robot.drivetrain.currentheading);
            telemetry.addData("Stone Sensor", distance);
            telemetry.addData("Lift Pos", robot.stacker.getLiftPosition());
            telemetry.addData("Arm Pos", robot.stacker.getArmPosition());
            telemetry.update();
        }
    }
}