package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    //Electronics
    private DcMotorEx leftIntake;
    private DcMotorEx rightIntake;

    //OpMode Related Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    //Constructor
    public Intake(HardwareMap hardwareMap, LinearOpMode op){

        this.op = op;
        this.hardwareMap = hardwareMap;

        leftIntake  = hardwareMap.get(DcMotorEx.class, "intake1");
        rightIntake = hardwareMap.get(DcMotorEx.class, "intake2");

        leftIntake.setDirection(DcMotor.Direction.FORWARD);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        op.telemetry.addLine("Status" + "Intake Initialized");
        op.telemetry.update();

    }

    public void setControls(double intakePower) {
        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);
    }

    public void setIntakePower(double power){
        leftIntake.setPower(power);
        rightIntake.setPower(power);
    }
}