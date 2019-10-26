package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "OpenCV")
public class skyStoneDetectionTest extends LinearOpMode {

    @Override public void runOpMode() {
        skyStoneDetector skyStoneDetector = new skyStoneDetector(this);
        skyStoneDetector.initializeCamera();

        waitForStart();
        skyStoneDetector.start();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("SkyStone X Location", skyStoneDetector.getPosition());
            telemetry.addData("# of SkyStones", skyStoneDetector.getNumberOfStones());
            telemetry.addData("Fps", skyStoneDetector.getFps());
            telemetry.update();
            idle();
        }
        skyStoneDetector.interrupt();
        telemetry.addData("Status", "Done"); telemetry.update();
    }
}