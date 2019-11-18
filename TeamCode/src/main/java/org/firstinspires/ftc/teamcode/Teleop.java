package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp(name="Teleop") @SuppressWarnings("FieldCanBeLocal")
public class Teleop extends LinearOpMode {

    private Robot robot;

    private double forward = 0;
    private double right = 0;
    private double angle = 0;
    private double intakePower = 1;
    private double transferPower = 0;
    private double clampPower = 0;

    private final boolean robotCentric = true;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, this, 0 , 0,0);
        robot.deposit.unclampStone();
        robot.clamp.openClamp();
        ElapsedTime xBuffer = new ElapsedTime();

        waitForStart();
        robot.drivetrain.resetAngle();
        robot.intake.setControls(1);

        while (opModeIsActive()){
            angle = robot.drivetrain.getAngle();
            if (robotCentric) {
            forward = gamepad1.left_stick_y;
            right = gamepad1.left_stick_x;
            } else {
                forward = gamepad1.left_stick_y * Math.sin(angle) - gamepad1.left_stick_x * Math.cos(angle);
                right = gamepad1.left_stick_y * Math.cos(angle) + gamepad1.left_stick_x * Math.sin(angle);
            }

            if (gamepad2.dpad_up) {intakePower += 0.2;}
            else if (gamepad2.dpad_down) {intakePower -= 0.2;}

            if (gamepad2.x && xBuffer.milliseconds()>1000 && intakePower == 0) {
                intakePower = 1;
                xBuffer.reset();
                xBuffer.startTime();
            } else if (gamepad2.x && xBuffer.milliseconds()>1000) {
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

            if (gamepad1.a) robot.clamp.openClamp();
            else if (gamepad1.b) robot.clamp.closeClamp();
            clampPower = gamepad1.right_trigger*0.25 - gamepad1.left_trigger*0.25;

            robot.drivetrain.setControls(right, forward, gamepad1.right_stick_x); robot.drivetrain.updatePose();
            robot.intake.setControls(intakePower);
            robot.deposit.setControls(transferPower);
            robot.clamp.setControls(clampPower);

            telemetry.addData("X", robot.drivetrain.x);
            telemetry.addData("Y", robot.drivetrain.y);
            telemetry.addData("Theta", robot.drivetrain.currentheading);
            telemetry.addData("Heading", angle);
            telemetry.addData("stone sensor", distance);
            telemetry.update();
        }
    }
}