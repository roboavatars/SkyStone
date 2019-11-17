package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.OpenCV.skyStoneDetector;
import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;

@Autonomous
public class AutoTest extends LinearOpMode {

    private double ssAligned = 125;
    private double curPos;
    private MecanumDrivetrain drivetrain;
    skyStoneDetector detector = new skyStoneDetector(this);

    @Override
    public void runOpMode() throws InterruptedException {
        //initializing skystone detector stuff after init
        detector.initializeCamera();
        detector.start();
        detector.setActive(true);
        detector.isAllianceRed(true);

        //initializing mecanum dt
        drivetrain = new MecanumDrivetrain(hardwareMap,this,0,0,0);

        while (!opModeIsActive() && !isStopRequested()) {
            curPos = detector.getPosition();
            if (Math.abs(ssAligned - curPos) < 5) {telemetry.addData("Status", "Ready to Get SkyStone");}

            telemetry.addData("SkyStone Position: ", curPos);
            telemetry.addData("SkyStone X Location", detector.getSSPosX());
            telemetry.addData("# of SkyStones", detector.getNumberOfStones());
            telemetry.update();
        }

        //after start
        waitForStart();


    }
}
