package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.old_stuff.Deposit;

@TeleOp(name="Deposit Test") @SuppressWarnings("FieldCanBeLocal")
@Disabled
public class DepositTest extends LinearOpMode {

    private Deposit deposit;
    private double depositPower = 0;

    @Override
    public void runOpMode() {
        deposit = new Deposit(this);
        deposit.clampStone();

        waitForStart();

        while(opModeIsActive()){

            if (gamepad2.a) deposit.clampStone();
            else if (gamepad2.b) deposit.unclampStone();
            depositPower = gamepad2.right_trigger - gamepad2.left_trigger;

            deposit.setControls(depositPower);

            telemetry.addData("Deposit Power", depositPower);
            telemetry.addData("Position", deposit.getDepositPos());
            telemetry.update();
        }
    }
}