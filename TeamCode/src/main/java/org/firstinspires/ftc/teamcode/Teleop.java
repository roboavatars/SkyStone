package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RobotClasses.Clamp;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Transfer;

@TeleOp(name="Teleop") @SuppressWarnings("FieldCanBeLocal")
public class Teleop extends LinearOpMode {

    private MecanumDrivetrain drivetrain;
    private Intake intake;
    private Transfer transfer;
    private Clamp clamp;

    private double forward = 0;
    private double right = 0;
    private double angle = 0;
    private double intakePower = 1;
    private double transferPower = 0;
    private double clampPower = 0;

    private final boolean robotCentric = true;

    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrivetrain(hardwareMap,this,0,0,0);
        intake = new Intake(hardwareMap,this);
        transfer = new Transfer(hardwareMap, this);
        clamp = new Clamp(hardwareMap,this);
        ElapsedTime xBuffer = new ElapsedTime();

        transfer.openTransfer();
        clamp.openClamp();

        waitForStart();
        drivetrain.resetAngle();
        intake.setControls(1);

        while(opModeIsActive()){
            angle = drivetrain.getAngle();
            if (robotCentric) {
            forward = gamepad1.left_stick_y;
            right = gamepad1.left_stick_x;
            } else {
                forward = gamepad1.left_stick_y * Math.sin(angle) - gamepad1.left_stick_x * Math.cos(angle);
                right = gamepad1.left_stick_y * Math.cos(angle) + gamepad1.left_stick_x * Math.sin(angle);
            }

            if (gamepad2.dpad_up) {intakePower += 0.2;}
            else if (gamepad2.dpad_down) {intakePower -= 0.2;}

            if (gamepad2.x && xBuffer.milliseconds()>1000 && intakePower==0) {
                intakePower = 1;
                xBuffer.reset();
                xBuffer.startTime();
            } else if (gamepad2.x && xBuffer.milliseconds()>1000) {
                intakePower = 0;
                xBuffer.reset();
                xBuffer.startTime();
            }

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
            telemetry.update();
        }
    }
}