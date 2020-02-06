package org.firstinspires.ftc.teamcode.Tests;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    public void runOpMode() {
        drivetrain = new MecanumDrivetrain(this,0,0,0, false);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        time.reset();
        while(opModeIsActive()){
            drivetrain.updatePose();
            TelemetryPacket packet = new TelemetryPacket();

            packet.put("x", drivetrain.x);
            packet.put("y", drivetrain.y);
            packet.put("theta", drivetrain.currentheading);
            dashboard.sendTelemetryPacket(packet);

            Log.w("auto", String.format("%.5f", drivetrain.x) + " " + String.format("%.5f", drivetrain.y) + " " + String.format("%.5f", drivetrain.currentheading));
        }
    }
}
