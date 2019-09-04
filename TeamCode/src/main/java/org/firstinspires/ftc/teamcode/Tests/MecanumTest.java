package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;

@TeleOp
public class MecanumTest extends LinearOpMode {

    MecanumDrivetrain drivetrain;
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new MecanumDrivetrain(hardwareMap,this,0,0,0);
        waitForStart();

        while(opModeIsActive()){
            drivetrain.setControls(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
            drivetrain.updatePose();

            telemetry.addData("x", drivetrain.x);
            telemetry.addData("y", drivetrain.y);
            telemetry.addData("theta", drivetrain.currentheading);
            telemetry.update();
        }
    }
}
