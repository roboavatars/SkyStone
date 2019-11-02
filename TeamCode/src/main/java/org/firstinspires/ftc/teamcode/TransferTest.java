package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotClasses.Transfer;

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="Transfer Test")
public class TransferTest extends LinearOpMode {

    private Transfer transfer;

    private double transferPower = 0;
    private double transferPosition = 0;

    @Override
    public void runOpMode() {
        transfer = new Transfer(hardwareMap, this);

        waitForStart();

        while(opModeIsActive()){

            if (gamepad2.dpad_up){transferPosition += 0.05;}
            if (gamepad2.dpad_down){transferPosition -= 0.05;}
            transferPosition = Range.clip(transferPosition, 0, 1);
            transferPower = gamepad2.right_trigger*0.25 - gamepad2.left_trigger*0.25;

            transfer.setControls(transferPower, transferPosition);

            telemetry.addData("Transfer Position", transferPosition);
            telemetry.addData("Transfer Lift Power", transferPower);
            telemetry.update();
        }
    }
}