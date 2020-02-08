package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Clamp Test") @SuppressWarnings("FieldCanBeLocal")
@Config
public class ClampTest extends LinearOpMode {

    private Servo clamp;
    public static double clampPos = 0.86;
    public static double unClampPos = 0.01;
    public static boolean open = true;

    @Override
    public void runOpMode() {

        clamp = hardwareMap.get(Servo.class, "stoneClamp");
        clamp.setPosition(unClampPos);

        waitForStart();

        while(opModeIsActive()) {

            if (open) {clamp.setPosition(unClampPos);}
            else {clamp.setPosition(clampPos);}

            telemetry.addData("Cur Position", clamp.getPosition());
            telemetry.update();
        }
    }
}