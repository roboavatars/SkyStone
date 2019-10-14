package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@SuppressWarnings("StatementWithEmptyBody")
@Autonomous(name = "OpenCV Example Auto")
public class OpenCVExampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        OpenCVBase vision = new OpenCVBase(this);
        vision.initializeCamera();

        waitForStart();

        vision.start();

        while (opModeIsActive() && !isStopRequested()) {
            while (!vision.ready()) {}
            telemetry.addData("SkyStonePosition", vision.position());
            telemetry.update();
            vision.interrupt();
        }

    }
}