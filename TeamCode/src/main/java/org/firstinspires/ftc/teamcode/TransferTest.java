package org.firstinspires.ftc.teamcode;

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
    private double transferPosition = 0;

    @Override
    public void runOpMode() {
        transfer = new Transfer(hardwareMap, this);
        transfer.setControls(0,0.88);

        waitForStart();

        while(opModeIsActive()){

            if (gamepad2.a) transferPosition = 0.91;
            else if (gamepad2.b) transferPosition = 0.88;
            transferPower = gamepad2.right_trigger - gamepad2.left_trigger;

            transfer.setControls(transferPower, transferPosition);

            telemetry.addData("Transfer Servo Power", transferPosition);
            telemetry.addData("Transfer Power", transferPower);
            telemetry.addData("Position", transfer.getTransferPos());
            telemetry.update();
        }
    }
}