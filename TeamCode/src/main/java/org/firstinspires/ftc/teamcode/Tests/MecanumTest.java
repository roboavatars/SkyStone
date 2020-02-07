package org.firstinspires.ftc.teamcode.Tests;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.Splines.Spline;
import org.firstinspires.ftc.teamcode.Splines.SplineGenerator;

@TeleOp
public class MecanumTest extends LinearOpMode {

    private MecanumDrivetrain drivetrain;
    private ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrivetrain(this,9,111,0, false);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        SplineGenerator splineGenerator = new SplineGenerator();
        Spline[] testSpline = splineGenerator.SplineBetween3Points(9,111,32,78,32,40,0,Math.PI/2,
                Math.PI/2,30,70,40,2,1);

        waitForStart();

        time.reset();
        while(opModeIsActive()){
            drivetrain.updatePose();
            TelemetryPacket packet = new TelemetryPacket();
            double currentTime = Math.min(2, time.seconds());

            drivetrain.setTargetPoint(testSpline[0].position(currentTime), testSpline[1].position(currentTime), 0);

            packet.put("x", drivetrain.x);
            packet.put("y", drivetrain.y);
            packet.put("theta", drivetrain.currentheading);
            dashboard.sendTelemetryPacket(packet);

            Log.w("auto", String.format("%.5f", drivetrain.x) + " " + String.format("%.5f", drivetrain.y) + " " + String.format("%.5f", drivetrain.currentheading));
        }
    }
}
