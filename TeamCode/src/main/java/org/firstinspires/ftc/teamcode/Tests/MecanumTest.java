package org.firstinspires.ftc.teamcode.Tests;

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
        drivetrain = new MecanumDrivetrain(hardwareMap,this,0,0,0);
        waitForStart();
        double thetaerror = 0;
        double xerror = 0;
        double yerror = 0;
        time.reset();
        while(opModeIsActive()){
            drivetrain.updatePose();
            xerror = drivetrain.x - 20*Math.cos(2*time.seconds());
            yerror = drivetrain.y - 20*Math.sin(2*time.seconds());
            thetaerror = drivetrain.currentheading - Math.PI/2;


//            drivetrain.setGlobalControls(-0.1*xerror,-0.1*yerror,-1*thetaerror);
//            drivetrain.setControls(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("xvelocity", -0.03*xerror);
            telemetry.addData("yvelocity", -0.03*yerror);

            telemetry.addData("x", drivetrain.x);
            telemetry.addData("y", drivetrain.y);
            telemetry.addData("theta", drivetrain.currentheading);
            telemetry.update();
        }
    }
}
