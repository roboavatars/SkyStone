package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "OpenCV")
public class detectorTest extends LinearOpMode {

    @Override public void runOpMode() {
        skyStoneDetector detector = new skyStoneDetector(this);
        detector.initializeCamera();
        detector.start();
        detector.setActive(true);
        telemetry.addData("Status", "Ready"); telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            double curPos = detector.getPosition();
            if (curPos > 120 && curPos < 130) { //125
                telemetry.addData("Status", "Ready to Get SkyStone");
            }
            telemetry.addData("SkyStone X Location", curPos);
            telemetry.addData("# of SkyStones", detector.getNumberOfStones());
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            double curPos = detector.getPosition();
            if (curPos > 120 && curPos < 130) { //125
                telemetry.addData("Status", "Ready to Get SkyStone");
            }
            telemetry.addData("SkyStone X Location", curPos);
            telemetry.addData("# of SkyStones", detector.getNumberOfStones());
            telemetry.update();
        }
        detector.setActive(false);
        telemetry.addData("Status", "Done"); telemetry.update();
    }
}