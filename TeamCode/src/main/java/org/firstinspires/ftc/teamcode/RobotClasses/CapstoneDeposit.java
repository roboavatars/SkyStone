package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class CapstoneDeposit {

    //Electronics
    private Servo capstoneDeposit;

    private final double out = 1;
    private final double tolerance = 0.1;

    //OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    public CapstoneDeposit(LinearOpMode op){

        this.op = op;
        this.hardwareMap = op.hardwareMap;

        capstoneDeposit = hardwareMap.get(Servo.class, "capstoneDeposit");

        op.telemetry.addData("Status", "Grabbers Initialized");
        op.telemetry.update();
    }

    public void retract() {
        capstoneDeposit.setPosition(1);
    }
    public void attachCapstone() {
        capstoneDeposit.setPosition(-1);
    }

    public double getGrabberPosition() {
        return capstoneDeposit.getPosition();
    }

    public boolean isOut() {
        return Math.abs(getGrabberPosition() - out) < tolerance;
    }
}
