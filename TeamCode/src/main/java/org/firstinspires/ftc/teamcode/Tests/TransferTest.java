package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotClasses.Transfer;

@TeleOp(name="Transfer Test") @SuppressWarnings("FieldCanBeLocal")
public class TransferTest extends LinearOpMode {

    private Transfer transfer;
    private double transferPower = 0;

    @Override
    public void runOpMode() {
        transfer = new Transfer(hardwareMap, this);
        transfer.openTransfer();

        waitForStart();

        while(opModeIsActive()){

            if (gamepad2.a) transfer.openTransfer();
            else if (gamepad2.b) transfer.closeTransfer();
            transferPower = gamepad2.right_trigger - gamepad2.left_trigger;

            transfer.setControls(transferPower);

            telemetry.addData("Transfer Power", transferPower);
            telemetry.addData("Position", transfer.getTransferPos());
            telemetry.update();
        }
    }
}