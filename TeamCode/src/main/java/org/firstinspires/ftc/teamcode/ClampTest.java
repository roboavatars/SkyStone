package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotClasses.Clamp;

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="Clamp Test")
public class ClampTest extends LinearOpMode {

    private Clamp clamp;

    private double clampPosition = 0;
    private double clampPower = 0;

    @Override
    public void runOpMode() {
        clamp = new Clamp(hardwareMap,this);

        waitForStart();
        clamp.setClampPosition(0);

        while(opModeIsActive()){

            if (gamepad1.dpad_right){clampPosition += 0.05;}
            if (gamepad1.dpad_left){clampPosition -= 0.05;}
            clampPosition = Range.clip(clampPosition, 0, 1);
            clampPower = gamepad1.right_trigger*0.25 - gamepad1.left_trigger*0.25;

            clamp.setControls(clampPosition, clampPower);

            telemetry.addData("Clamp Position", clampPosition);
            telemetry.addData("Clamp Lift Power", clampPower);
            telemetry.update();
        }
    }
}