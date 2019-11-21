package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class FoundationGrabber {

    //Electronics
    private Servo leftGrabber;
    private Servo rightGrabber;

    //OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    public FoundationGrabber(LinearOpMode op){

        this.op = op;
        this.hardwareMap = op.hardwareMap;

        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");

        op.telemetry.addData("Status", "Grabbers Initialized");
        op.telemetry.update();
    }

    public void releaseFoundation() {
        leftGrabber.setPosition(1);
        rightGrabber.setPosition(-1);
    }

    public void grabFoundation() {
        leftGrabber.setPosition(-1);
        rightGrabber.setPosition(1);
    }
}
