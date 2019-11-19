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
    private final double leftClose = 0.72;
    private final double rightClose = 0.33;
    private final double leftOpen = 0.15;
    private final double rightOpen = 0.9;

    //OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    public Clamp(LinearOpMode op){

        this.op = op;
        this.hardwareMap = op.hardwareMap;

        clampLift = hardwareMap.get(DcMotorEx.class, "clampLift");
        leftClamp = hardwareMap.get(Servo.class, "leftClamp");
        rightClamp = hardwareMap.get(Servo.class, "rightClamp");

        clampLift.setDirection(DcMotor.Direction.FORWARD);
        clampLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        op.telemetry.addData("Status", "Clamp Initialized");
        op.telemetry.update();
    }

    public void setControls(double liftPower) {
        clampLift.setPower(liftPower);
    }

    public void openClamp() {
        leftClamp.setPosition(leftOpen);
        rightClamp.setPosition(rightOpen);
    }

    public void closeClamp() {
        leftClamp.setPosition(leftClose);
        rightClamp.setPosition(rightClose);
    }

    public void setLiftPower(double power){clampLift.setPower(power);}

    public double getLPosition() {
        return leftClamp.getPosition();
    }
    public double getRPosition() {
        return rightClamp.getPosition();
    }
}
