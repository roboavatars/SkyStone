package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Clamp;

@TeleOp(name="Clamp Test") @SuppressWarnings("FieldCanBeLocal")
public class ClampTest extends LinearOpMode {

    private Clamp clamp;
    private double clampPower = 0;
    private double spControl = 0.375;

    @Override
    public void runOpMode() {
        clamp = new Clamp(hardwareMap,this);
        clamp.openClamp();

        waitForStart();

        while(opModeIsActive()){

            if (gamepad1.a) clamp.openClamp();
            else if (gamepad1.b) clamp.closeClamp();
            clampPower = gamepad1.right_trigger*spControl - gamepad1.left_trigger*spControl;

            clamp.setControls(clampPower);

            telemetry.addData("Clamp Lift Power", clampPower);
            telemetry.addData("L Clamp Position", clamp.getLPosition());
            telemetry.addData("R Clamp Position", clamp.getRPosition());
            telemetry.update();
        }
    }
}