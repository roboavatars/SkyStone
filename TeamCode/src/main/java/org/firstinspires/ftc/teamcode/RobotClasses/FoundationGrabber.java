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

    private final double grabPos = 0;
    private final double releasePos = 0;

    //OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    public FoundationGrabber(HardwareMap hardwareMap, LinearOpMode op){

        this.op = op;
        this.hardwareMap = hardwareMap;

        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");

        op.telemetry.addData("Status", "Grabbers Initialized");
        op.telemetry.update();
    }

    public void grabFoundation() {
        leftGrabber.setPosition(grabPos);
        rightGrabber.setPosition(grabPos);
    }

    public void releaseFoundation() {
        leftGrabber.setPosition(releasePos);
        rightGrabber.setPosition(releasePos);
    }

    public double getLPosition() {
        return leftGrabber.getPosition();
    }
    public double getRPosition() {
        return rightGrabber.getPosition();
    }
}
