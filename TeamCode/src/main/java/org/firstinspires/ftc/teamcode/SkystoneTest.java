package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotClasses.Clamp;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;

import org.firstinspires.ftc.teamcode.OpenCV.skyStoneDetector;

@SuppressWarnings("FieldCanBeLocal")
@Autonomous(name = "Auto Test")
public class SkystoneTest extends LinearOpMode {

    private MecanumDrivetrain drivetrain;
    private Intake intake;
    private Clamp clamp;
    private skyStoneDetector detector;

    @Override
    public void runOpMode() {
        drivetrain = new MecanumDrivetrain(hardwareMap,this,0,0,0);
        intake = new Intake(hardwareMap,this);
        clamp = new Clamp(hardwareMap,this);
        detector = new skyStoneDetector(this);
        detector.initializeCamera();

        waitForStart();
        detector.start();
        drivetrain.resetAngle();
        clamp.openClamp();

        while (opModeIsActive() && !isStopRequested()) {
            drivetrain.setControls(-0.005*(70-detector.getPosition()),0,0);
            telemetry.addData("Position", detector.getPosition());
            telemetry.update();
        }

        drivetrain.setControls(0,0,0);
        telemetry.addData("Position", "Done");
        telemetry.update();

        //drivetrain.setControls(0,0.4,0);
        //sleep(2000);
        //intake.setControls(1);
        //sleep(1000);
        //drivetrain.setControls(0,0,0);
        //intake.setControls(0);

        telemetry.addData("Status", "Done");
        telemetry.update();
    }
}