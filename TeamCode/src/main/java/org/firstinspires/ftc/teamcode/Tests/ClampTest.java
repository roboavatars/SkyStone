package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Clamp Test") //@Config
public class ClampTest extends LinearOpMode {

    private Servo clamp;
    public static double stonePushPos = 0;
    public static double homePos = 1;
    public static boolean home = true;

    @Override
    public void runOpMode() {

        clamp = hardwareMap.get(Servo.class, "stonePushServo");
        clamp.setPosition(homePos);

        waitForStart();

        while(opModeIsActive()) {

            if (home) {clamp.setPosition(homePos);}
            else {clamp.setPosition(stonePushPos);}

            telemetry.addData("Cur Position", clamp.getPosition());
            telemetry.update();
        }
    }
}