package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsAnalogOpticalDistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Distance Sensor Test")
public class DistanceSensorTest extends LinearOpMode {

    private ModernRoboticsAnalogOpticalDistanceSensor stoneSensor;

    @Override
    public void runOpMode(){

        stoneSensor = hardwareMap.get(ModernRoboticsAnalogOpticalDistanceSensor.class, "stoneSensor");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("light", stoneSensor.getLightDetected());
            telemetry.addData("light raw", stoneSensor.getRawLightDetected());
            telemetry.update();
        }

    }
}
