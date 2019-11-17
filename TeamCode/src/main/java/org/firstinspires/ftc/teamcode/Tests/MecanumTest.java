package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;

@TeleOp
public class MecanumTest extends LinearOpMode {

    MecanumDrivetrain drivetrain;
    ElapsedTime time = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain = new MecanumDrivetrain(hardwareMap,this,9,111,0);
        waitForStart();
        double thetaerror = 0;
        double xerror = 0;
        double yerror = 0;
        time.reset();
        while(opModeIsActive()){
            drivetrain.updatePose();
            drivetrain.setTargetPoint(48,120,5*Math.PI/12);

            telemetry.addData("x", drivetrain.x);
            telemetry.addData("y", drivetrain.y);
            telemetry.addData("theta", drivetrain.currentheading);
            telemetry.update();
        }
    }
}
