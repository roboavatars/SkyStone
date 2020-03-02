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
        robot = new Robot(this,9,111, 0, true);
        waitForStart();

        while(opModeIsActive()){

            //robot.drivetrain.setTargetPointAuto(36,81, 0, 0.1, 0.1, 0.8);
//            robot.addPacket("lift ticks", robot.stacker.getLiftPosition());
            robot.drivetrain.setControls(0.6,0,-0.4);
            robot.update();

            Log.w("auto", String.format("%.5f", robot.drivetrain.x) + " " + String.format("%.5f", robot.drivetrain.y) + " " + String.format("%.5f", robot.drivetrain.currentheading));
        }
    }
}
