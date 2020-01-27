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
        drivetrain = new MecanumDrivetrain(this,25,9,3*Math.PI/2);
        waitForStart();
        double thetaerror = 0;
        double xtarg = 0;
        double ytarg = 0;
        time.reset();
        while(opModeIsActive()){
            drivetrain.updatePose();
            if(time.seconds()<Math.PI){
                xtarg = (23*Math.sin(time.seconds())+25);
                ytarg = (-23*Math.cos(time.seconds())+32);

            }
            else{
                xtarg = 25;
                ytarg = 75;
            }
            drivetrain.setTargetPoint(xtarg,ytarg, 3*Math.PI/2);

            telemetry.addData("x", drivetrain.x);
            telemetry.addData("y", drivetrain.y);
            telemetry.addData("theta", drivetrain.currentheading);
            telemetry.update();
        }
    }
}
