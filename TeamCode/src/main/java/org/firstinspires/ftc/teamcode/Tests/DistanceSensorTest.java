package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Distance Sensor Test")
public class DistanceSensorTest extends LinearOpMode {

    private Rev2mDistanceSensor stoneSensor;

    @Override
    public void runOpMode(){

        stoneSensor = hardwareMap.get(Rev2mDistanceSensor.class, "stoneSensor");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Distance Sensor", stoneSensor.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }

    }
}
