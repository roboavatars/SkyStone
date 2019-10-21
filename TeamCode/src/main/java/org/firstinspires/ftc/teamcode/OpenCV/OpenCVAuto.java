package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@SuppressWarnings("StatementWithEmptyBody")
@Autonomous(name = "OpenCV")
public class OpenCVAuto extends LinearOpMode {

    @Override public void runOpMode() {
        OpenCVBase skyStoneDetector = new OpenCVBase(this);
        skyStoneDetector.initializeCamera();

        waitForStart();

        skyStoneDetector.start();

        if (opModeIsActive() && !isStopRequested()) {
            while (!skyStoneDetector.isReady());
            telemetry.addData("SkyStone X Location", skyStoneDetector.getPosition());
            telemetry.addData("OpenCV FPS", skyStoneDetector.getFps());
            skyStoneDetector.interrupt();
            telemetry.addData("Status", "Done");
            telemetry.update();
        }
        sleep(2000);
    }
}