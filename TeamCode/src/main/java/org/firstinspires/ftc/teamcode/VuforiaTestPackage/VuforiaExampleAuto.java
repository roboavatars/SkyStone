package org.firstinspires.ftc.teamcode.VuforiaTestPackage;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Vuforia Example Auto")
public class VuforiaExampleAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        VuforiaLocator engine = new VuforiaLocator(this);

        waitForStart();

        engine.start();

        while (opModeIsActive()) {
            telemetry.addData("X", engine.getVuforiaX());
            telemetry.addData("Y", engine.getVuforiaY());
            telemetry.addData("Theta", engine.getVuforiaTheta());
            telemetry.update();
            sleep(1000);
        }

        engine.interrupt();
    }
}