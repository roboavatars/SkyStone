package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RobotClasses.Stacker;

@TeleOp(name="Stacker Test") @SuppressWarnings("FieldCanBeLocal")
public class StackerTest extends LinearOpMode {

    private Stacker stacker;

    @Override
    public void runOpMode() {
        stacker = new Stacker(this);

        waitForStart();

        while(opModeIsActive()){
            stacker.setLiftPower(gamepad1.left_stick_y * 0.75);
            stacker.setDepositPower(gamepad1.right_stick_y * 0.75);

            if (gamepad1.y) {stacker.clampStone();}
            else if (gamepad1.a) {stacker.unClampStone();}
        }
    }
}