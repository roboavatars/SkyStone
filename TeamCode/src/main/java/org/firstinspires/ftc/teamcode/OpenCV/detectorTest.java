package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "OpenCV")
public class detectorTest extends LinearOpMode {

    @Override public void runOpMode() {

        skyStoneDetector detector = new skyStoneDetector(this);
        detector.initializeCamera();
        detector.start();
        detector.setAutoActive(true);
        telemetry.addData("Status", "Ready"); telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("SkyStone X Location", detector.getPosition());
            telemetry.addData("# of SkyStones", detector.getNumberOfStones());
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("SkyStone X Location", detector.getPosition());
            telemetry.addData("# of SkyStones", detector.getNumberOfStones());
            telemetry.update();
        }
        detector.setAutoActive(false);
        telemetry.addData("Status", "Done"); telemetry.update();
    }
}