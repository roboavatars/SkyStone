package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotClasses.Clamp;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Transfer;

@TeleOp(name="Robot Centric Teleop") @SuppressWarnings("FieldCanBeLocal")
public class RobotTeleop extends LinearOpMode {

    private MecanumDrivetrain drivetrain;
    private Intake intake;
    private Transfer transfer;
    private Clamp clamp;

    private double forward = 0;
    private double right = 0;
    private double angle = 0;
    private double intakePower = 0;
    private double transferPower = 0;
    private double clampPower = 0;

    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrivetrain(hardwareMap,this,0,0,0);
        intake = new Intake(hardwareMap,this);
        transfer = new Transfer(hardwareMap, this);
        clamp = new Clamp(hardwareMap,this);

        transfer.openTransfer();
        clamp.openClamp();

        waitForStart();
        drivetrain.resetAngle();

        while(opModeIsActive()){
            angle = drivetrain.getAngle();
            forward = gamepad1.left_stick_y;
            right = gamepad1.left_stick_x;

            //if (gamepad1.a) intakePower = 1;
            //else intakePower = 0;

            if (gamepad2.a) transfer.openTransfer();
            else if (gamepad2.b) transfer.closeTransfer();
            transferPower = gamepad2.right_trigger - gamepad2.left_trigger;

            if (gamepad1.a) clamp.openClamp();
            else if (gamepad1.b) clamp.closeClamp();
            clampPower = gamepad1.right_trigger*0.25 - gamepad1.left_trigger*0.25;

            drivetrain.setControls(right, forward, gamepad1.right_stick_x); drivetrain.updatePose();
            intake.setControls(intakePower);
            transfer.setControls(transferPower);
            clamp.setControls(clampPower);

            telemetry.addData("X", drivetrain.x);
            telemetry.addData("Y", drivetrain.y);
            telemetry.addData("Theta", drivetrain.currentheading);
            telemetry.addData("Heading", angle);
            telemetry.addData("Transfer Lift Power", transferPower);
            telemetry.addData("Clamp Lift Power", clampPower);
            telemetry.update();
        }
    }
}