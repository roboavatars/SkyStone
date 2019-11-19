package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Intake {

    //Electronics
    private DcMotorEx leftIntake;
    private DcMotorEx rightIntake;

    //OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    public Intake(LinearOpMode op){

        this.op = op;
        this.hardwareMap = op.hardwareMap;

        leftIntake = hardwareMap.get(DcMotorEx.class, "intake1");
        rightIntake = hardwareMap.get(DcMotorEx.class, "intake2");

        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        op.telemetry.addData("Status", "Intake Initialized");
        op.telemetry.update();
    }

    public void setControls(double intakePower) {
        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);
    }
}