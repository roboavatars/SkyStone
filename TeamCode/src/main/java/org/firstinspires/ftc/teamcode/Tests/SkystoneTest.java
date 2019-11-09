package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.RobotClasses.Clamp;
import org.firstinspires.ftc.teamcode.RobotClasses.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;

import org.firstinspires.ftc.teamcode.OpenCV.skyStoneDetector;

@Autonomous(name = "Skystone Auto Test") @SuppressWarnings("FieldCanBeLocal")
public class SkystoneTest extends LinearOpMode {

    private MecanumDrivetrain drivetrain;
    private Intake intake;
    private Clamp clamp;
    private skyStoneDetector detector;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initializing, Please Wait!!!!!"); telemetry.update();
        drivetrain = new MecanumDrivetrain(hardwareMap,this,0,0,0);
        intake = new Intake(hardwareMap,this);
        clamp = new Clamp(hardwareMap,this);

        clamp.openClamp();
        detector = new skyStoneDetector(this);
        detector.initializeCamera();

        telemetry.addData("Status", "Ready"); telemetry.update();
        waitForStart();
        detector.start();
        drivetrain.resetAngle();

        drivetrain.setControls(0,-0.2,0); sleep(200);
        drivetrain.setControls(0,0,0);

        while (opModeIsActive() && !isStopRequested()) {
            double curPos = detector.getPosition();
            if (curPos < 60) curPos = 90;

            drivetrain.setControls(0.01*(curPos-70),0,0);
            if (curPos > 65 && curPos < 75) {
                telemetry.addData("Status", "Robot In Position"); telemetry.update();
                break;
            }
            telemetry.addData("Position", detector.getPosition()); telemetry.update();
        }

        drivetrain.setControls(0,0,0); sleep(100);
        drivetrain.setControls(0,-1,0); sleep(5000);
        intake.setControls(1); sleep(2000);
        drivetrain.setControls(0,0,0);
        intake.setControls(0);

        telemetry.addData("Status", "Done");
        telemetry.update();
    }
}