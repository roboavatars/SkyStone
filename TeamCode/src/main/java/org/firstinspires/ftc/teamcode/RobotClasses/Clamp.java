package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Clamp {
    //Electronics
    private DcMotorEx clampLift;
    private Servo leftClamp;
    private Servo rightClamp;

    //OpMode Related Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    //Constructor
    public Clamp(HardwareMap hardwareMap, LinearOpMode op){

        this.op = op;
        this.hardwareMap = hardwareMap;

        clampLift = hardwareMap.get(DcMotorEx.class, "clampLift");
        leftClamp = hardwareMap.get(Servo.class, "leftClamp");
        rightClamp = hardwareMap.get(Servo.class, "rightClamp");

        clampLift.setDirection(DcMotor.Direction.FORWARD);
        clampLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        op.telemetry.addLine("Status" + "Clamp Initialized");
        op.telemetry.update();

    }

    public void setControls(double clampPosition, double clampPower) {
        leftClamp.setPosition(clampPosition);
        rightClamp.setPosition(1 - clampPosition);
        clampLift.setPower(clampPower);
    }

    public void setClampPosition(double position){
        leftClamp.setPosition(position);
        rightClamp.setPosition(1 - position);
    }

    public void setLiftPower(double power){clampLift.setPower(power);}
}