package org.firstinspires.ftc.teamcode.OpenCV;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "OpenCV")
public class OpenCVAuto extends LinearOpMode {

    @Override public void runOpMode() {
        OpenCVBase skyStoneDetector = new OpenCVBase(this);
        skyStoneDetector.initializeCamera();

        waitForStart();

        skyStoneDetector.start();

        while (opModeIsActive() && !isStopRequested()) {
            //noinspection StatementWithEmptyBody
            while (!skyStoneDetector.ready());
            skyStoneDetector.interrupt();
            telemetry.addData("SkyStonePosition", skyStoneDetector.position());
            telemetry.update();
        }

    }
}