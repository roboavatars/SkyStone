package org.firstinspires.ftc.teamcode.AutoPrograms;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.skyStoneDetector;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.Splines.Path;
import org.firstinspires.ftc.teamcode.Splines.Pose;
import org.firstinspires.ftc.teamcode.Splines.Spline;
import org.firstinspires.ftc.teamcode.Splines.SplineGenerator;
import org.firstinspires.ftc.teamcode.Splines.Waypoint;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous (name = "Path based auto")
public class newPathBasedAuto extends LinearOpMode {

    private Robot robot;
    private skyStoneDetector detector = new skyStoneDetector(this);

    @Override
    public void runOpMode() {

        // Initialize Skystone Detector
        detector.initializeCamera();
        detector.start();
        detector.setActive(true);
        detector.isAllianceRed(true);

        // Initialize Robot
        robot = new Robot(this, 9, 111, 0, true);
        robot.logger.startLogging();
        robot.grabber.releaseFoundation();
        robot.intake.setControls(0);
        robot.stacker.unClampStone();
        robot.stacker.goHome();

        // Segment Finished Variables
        boolean skystone1 = false;
        boolean approachFoundation = false;
        boolean foundationPull = false;
        boolean skystone2 = false;
        boolean toFoundation2 = false;
        boolean toTape = false;



        // After Start
        waitForStart();

        // Skystone Position Variables
        double skystonePos = detector.getPosition();
        double skystoneY = robot.drivetrain.y;
        if (skystonePos == 1) {
            skystoneY = 128;

        }
        else if (skystonePos == 2) {
            skystoneY = 120;
        }
        else if (skystonePos == 3) {
            skystoneY = 112;
        }

        //stoppin detector
        detector.setActive(false);

        //Path time variables
        double skystone1Time = 1.5;

        //Defining paths
        Waypoint[] skystone1PathWaypoints = {
                new Waypoint(9,111,0,15,100,1.5,0),
//                 new Waypoint(33, skystoneY-4, Math.PI/6, 50,  60, 1.5,1),
                new Waypoint(45,skystoneY,Math.PI/4+0.2,10,-100, 0,skystone1Time)
        };
        Path skystone1Path = new Path(new ArrayList<>(Arrays.asList(skystone1PathWaypoints)));

        Path toFoundation1Path = null;

        // Time Used to End Segments After a Certain Period of Time
        ElapsedTime time = new ElapsedTime();

        robot.intake.setControls(1.0);
        sleep(33);
        log("Skystone 1");

        // Robot Move Loop
        while (opModeIsActive()) {

            // Update Robot's Location and States
            robot.update();

            // Get the First Skystone
            if (!skystone1) {
                double currentTime = Math.min(skystone1Time, time.seconds());
                Pose robotpose = skystone1Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPoint(robotpose.getX(),robotpose.getY(),robotpose.getTheta());
                if(robot.stoneInRobot){
                    //setting variable to move on from this segment
                    skystone1 = true;

                    //defining the tofoundation path
                    Waypoint[] tofoundation1PathWaypoints = {
                            robot.currentRobotWaypoint(),

                    };
                    toFoundation1Path = new Path(new ArrayList<>(Arrays.asList(tofoundation1PathWaypoints)));
                }
            }

//            // Approach and Align Robot with Foundation
//            else if (!approachFoundation) {
//
//
//            }
//
//            // Turn and Pull Foundation
//            else if (!foundationPull) {
//
//            }
//
//
//            // Get Second Skystone
//            else if (!skystone2) {
//
//            }
//
//            // Go to Foundation to Deposit Second Skystone
//            else if (!toFoundation2) {
//
//            }
//
//            // Park Under Skybridge
//            else if (!toTape) {
//                robot.drivetrain.setTargetPoint(30, 72, Math.PI / 2);
//
//            }
            else {
                robot.drivetrain.setControls(0,0,0);
            }

            //adding relevant information to telemetry
            robot.addPacket("Battery Voltage", getBatteryVoltage());
            robot.addPacket("Time", (System.currentTimeMillis()-robot.startTime)/1000);
            robot.addPacket("Skystone Position", skystonePos);
        }

        robot.update();
        robot.logger.flush();
        robot.logger.stopLogging();
        detector.interrupt();
    }

    public void log(String message) {
        Log.w("auto", message);
    }

    public double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}