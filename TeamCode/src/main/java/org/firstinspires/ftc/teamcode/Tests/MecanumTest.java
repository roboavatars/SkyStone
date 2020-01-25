package org.firstinspires.ftc.teamcode.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;

@TeleOp
public class MecanumTest extends LinearOpMode {

    private MecanumDrivetrain drivetrain;
    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new MecanumDrivetrain(this,0,0,0, false);

        waitForStart();

        time.reset();
        while(opModeIsActive()){
            drivetrain.updatePose();

            telemetry.addData("x", drivetrain.x);
            telemetry.addData("y", drivetrain.y);
            telemetry.addData("theta", drivetrain.currentheading);
            telemetry.update();

            Log.w("auto", String.format("%.5f", drivetrain.x) + " " + String.format("%.5f", drivetrain.y) + " " + String.format("%.5f", drivetrain.currentheading));
        }
    }
}
