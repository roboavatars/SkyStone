package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Clamp Test") @SuppressWarnings("FieldCanBeLocal")
public class ClampTest extends LinearOpMode {

    private Servo clamp;
    private double clampPos = 0.5;
    private double unClampPos = 1;

    @Override
    public void runOpMode() {

        clamp = hardwareMap.get(Servo.class, "stoneClamp");
        clamp.setPosition(unClampPos);

        waitForStart();

        while(opModeIsActive()) {

            if (gamepad1.a) clamp.setPosition(clampPos);
            else if (gamepad1.b) clamp.setPosition(unClampPos);

            telemetry.addData("Cur Position", clamp.getPosition());
            telemetry.update();
        }
    }
}