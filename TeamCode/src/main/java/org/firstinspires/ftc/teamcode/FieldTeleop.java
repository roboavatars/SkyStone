package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotClasses.Clamp;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;

@SuppressWarnings("FieldCanBeLocal")
@TeleOp(name="Field Centric Teleop") @Disabled
public class FieldTeleop extends LinearOpMode {

    private MecanumDrivetrain drivetrain;
    private Intake intake;
    private Clamp clamp;

    private double intakePower = 1;
    private double clampPosition = 0;
    private double angle = 0;
    private double forward = 0;
    private double right = 0;
    private double clampPower = 0;

    @Override
    public void runOpMode() {
        /*drivetrain = new MecanumDrivetrain(hardwareMap,this,0,0,0);
        intake = new Intake(hardwareMap,this);
        clamp = new Clamp(hardwareMap,this);

        waitForStart();
        drivetrain.resetAngle();
        clamp.setClampPosition(0);

        while(opModeIsActive()){
            angle = drivetrain.getAngle();
            forward = gamepad1.left_stick_y * Math.sin(angle) - gamepad1.left_stick_x * Math.cos(angle);
            right = gamepad1.left_stick_y * Math.cos(angle) + gamepad1.left_stick_x * Math.sin(angle);
            clampPower = gamepad1.right_trigger*0.25-gamepad1.left_trigger*0.25;

            if (gamepad1.dpad_up){intakePower += 0.05;}
            if (gamepad1.dpad_down){intakePower -= 0.05;}
            intakePower = Range.clip(intakePower, 0, 1);

            if (gamepad1.dpad_right){clampPosition += 0.05;}
            if (gamepad1.dpad_left){clampPosition -= 0.05;}
            clampPosition = Range.clip(clampPosition, 0, 1);

            drivetrain.setControls(right, forward, gamepad1.right_stick_x);
            drivetrain.updatePose();
            intake.setControls(intakePower);
            clamp.setControls(clampPosition, clampPower);

            telemetry.addData("X", drivetrain.x);
            telemetry.addData("Y", drivetrain.y);
            telemetry.addData("Theta", drivetrain.currentheading);
            telemetry.addData("Intake Power", intakePower);
            telemetry.addData("Heading", angle);
            telemetry.addData("Clamp Position", clampPosition);
            telemetry.addData("Clamp Lift Power", clampPower);
            telemetry.update();
        }*/
    }
}