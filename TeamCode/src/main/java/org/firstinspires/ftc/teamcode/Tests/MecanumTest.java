package org.firstinspires.ftc.teamcode.Tests;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp
public class MecanumTest extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(this,9,111, 0, false, false);
        waitForStart();

        while(opModeIsActive()){
            robot.update();
            Log.w("auto", String.format("%.5f", robot.drivetrain.x) + " " + String.format("%.5f", robot.drivetrain.y) + " " + String.format("%.5f", robot.drivetrain.currentheading));
        }
    }
}
