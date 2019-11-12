package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
        detector.start();
        detector.setActive(true);
        telemetry.addData("Status", "Ready"); telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Position", detector.getPosition()); telemetry.update();
        }
        waitForStart();
        drivetrain.resetAngle();
        drivetrain.setControls(0,-0.2,0); sleep(200);
        drivetrain.setControls(0,0,0);

        while (opModeIsActive() && !isStopRequested()) {
            double curPos = detector.getPosition();

            drivetrain.setControls(0.01*(curPos-125),0,0);
            if (curPos > 120 && curPos < 130) {
                telemetry.addData("Status", "Robot In Position"); telemetry.update();
                break;
            }
            telemetry.addData("Position", detector.getPosition()); telemetry.update();
        }

        detector.setActive(false);
        drivetrain.setControls(0,0,0); sleep(100);
        drivetrain.setControls(0,-1,0); sleep(5000);
        intake.setControls(1); sleep(2000);
        drivetrain.setControls(0,0,0);
        intake.setControls(0);

        telemetry.addData("Status", "Done");
        telemetry.update();
    }
}