package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Clamp {

    //Electronics
    private DcMotorEx clampLift;
    private Servo leftClamp;
    private Servo rightClamp;

    //left decreases outward, right increases outward
    private final double leftPosClose = 0.72;
    private final double rightPosClose = 0.33;
    private final double leftPosOpen = 0.05;
    private final double rightPosOpen = 0.9;

    //OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    public Clamp(HardwareMap hardwareMap, LinearOpMode op){

        this.op = op;
        this.hardwareMap = hardwareMap;

        clampLift = hardwareMap.get(DcMotorEx.class, "clampLift");
        leftClamp = hardwareMap.get(Servo.class, "leftClamp");
        rightClamp = hardwareMap.get(Servo.class, "rightClamp");

        clampLift.setDirection(DcMotor.Direction.FORWARD);
        clampLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        op.telemetry.addData("Status", "Clamp Initialized");
        op.telemetry.update();
    }

    public void setControls(double clampPower) {
        clampLift.setPower(clampPower);
    }

    public void closeClamp() {
        leftClamp.setPosition(leftPosClose);
        rightClamp.setPosition(rightPosClose);
    }

    public void openClamp() {
        leftClamp.setPosition(leftPosOpen);
        rightClamp.setPosition(rightPosOpen);
    }

    public void setLiftPower(double power){clampLift.setPower(power);}

    public double getLPosition() {
        return leftClamp.getPosition();
    }
    public double getRPosition() {
        return rightClamp.getPosition();
    }
}