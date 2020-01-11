package org.firstinspires.ftc.teamcode.RobotClasses;

import android.util.Log;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@SuppressWarnings("FieldCanBeLocal")
public class FoundationGrabber {

    //Electronics
    private Servo leftGrabber;
    private Servo rightGrabber;
    private Rev2mDistanceSensor autoAlignSensor;

    private final double grabberHome = 1;
    private final double grabberTolerance = 0.1;

    //OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    public FoundationGrabber(LinearOpMode op){

        this.op = op;
        this.hardwareMap = op.hardwareMap;

        leftGrabber = hardwareMap.get(Servo.class, "leftGrabber");
        rightGrabber = hardwareMap.get(Servo.class, "rightGrabber");
        autoAlignSensor = hardwareMap.get(Rev2mDistanceSensor.class, "autoAlignSensor");

        autoAlignSensor.initialize();

        op.telemetry.addData("Status", "Grabbers Initialized");
        op.telemetry.update();
    }

    public void releaseFoundation() {
        Log.w("graph", "release foundation called");
        leftGrabber.setPosition(1);
        rightGrabber.setPosition(-1);
    }
    public void grabFoundation() {
        Log.w("graph", "grab foundation called");
        leftGrabber.setPosition(-0.95);
        rightGrabber.setPosition(0.95);
    }

    public void extendRangeSensor(){
        rightGrabber.setPosition(0.75);
    }
    public double getDistance(){
        return autoAlignSensor.getDistance(DistanceUnit.INCH);
    }

    public double getGrabberPosition() {
        return rightGrabber.getPosition();
    }

    public boolean isGrabberHome() {
        return Math.abs(getGrabberPosition() - grabberHome) < grabberTolerance;
    }
}
