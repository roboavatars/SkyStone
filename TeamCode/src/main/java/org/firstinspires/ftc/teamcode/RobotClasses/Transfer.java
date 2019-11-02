package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Transfer {

    //Electronics
    private DcMotorEx transferMotor;
    private Servo transferServo;

    //OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    public Transfer(HardwareMap hardwareMap, LinearOpMode op){

        this.op = op;
        this.hardwareMap = hardwareMap;

        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        transferServo = hardwareMap.get(Servo.class, "transferServo");

        transferMotor.setDirection(DcMotor.Direction.FORWARD);

        op.telemetry.addData("Status", "Transfer Initialized");
        op.telemetry.update();
    }

    public void setControls(double motorPower, double servoPosition) {
        transferMotor.setPower(motorPower);
        transferServo.setPosition(servoPosition);
    }

    public double getTransferPos() {
        return transferServo.getPosition();
    }
}
