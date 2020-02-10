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

@Autonomous (name = "2 block New Auto")
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
        boolean toFoundation1 = false;
        boolean approachFoundation = false;
        boolean foundationPull = false;
        boolean skystone2 = false;
        boolean toFoundation2 = false;
        boolean toTape = false;

        //Path time variables
        double skystone1Time = 1.5;
        double toFoundation1Time = 3;
        double approachFoundationTime = 1.5;
        double foundationPullTime = 2.25;
        double skystone2Time = 4;
        double toFoundation2Time = 3;

        //deposit position
        double depositX, depositY, depositTheta;

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

        //stop detector
        detector.setActive(false);

        //Defining paths
        Waypoint[] skystone1PathWaypoints = { //turn first, then move towards block
                new Waypoint(9,111,0,20,100,0,0),
                new Waypoint(45,skystoneY,Math.PI / 4 + 0.2, 20, -100, 0, skystone1Time)
        };
        Path skystone1Path = new Path(new ArrayList<>(Arrays.asList(skystone1PathWaypoints)));
        Spline skystone1ThetaSpline = new Spline(0, 0, 8, Math.PI / 4 + 0.2, 0, 0, skystone1Time);

        Path toFoundation1Path = null;
        //Path approachFoundationPath = null;
        Path skystone2Path = null;
        Path toFoundation2Path = null;

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
                Pose robotPose = skystone1Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPoint(robotPose.getX(),robotPose.getY(),skystone1ThetaSpline.position(currentTime));

                if (robot.stoneInRobot || time.seconds() > skystone1Time+0.5) {
                    //setting variable to move on from this segment
                    skystone1 = true;
                    //reset time
                    time.reset();
                    //defining the tofoundation path
                    Waypoint[] toFoundation1PathWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.currentheading, -30, -100,0, 0),
                            new Waypoint(36, skystoneY - 25, Math.PI / 3, -50, -50,0, 1),
                            new Waypoint(36, 35, Math.PI / 2, -30, -50,0, 2),
                            new Waypoint(47, 18, Math.PI, -30, 100,0, toFoundation1Time)
                    };
                    toFoundation1Path = new Path(new ArrayList<>(Arrays.asList(toFoundation1PathWaypoints)));
                }
            }

            // Go to Foundation to Deposit First Skystone
            else if (!toFoundation1) {
                double currentTime = Math.min(toFoundation1Time, time.seconds());
                Pose robotPose = toFoundation1Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPoint(robotPose.getX(),robotPose.getY(),robotPose.getTheta()+Math.PI);

                if (time.seconds() > toFoundation1Time+0.5) {
                    robot.grabber.grabFoundation();
                }
                if (time.seconds() > toFoundation1Time + 1) {
                    toFoundation1 = true;
                    time.reset();
                }
            }

            // Turn and Pull Foundation
            else if (!foundationPull) {
                if (robot.drivetrain.currentheading > 2*Math.PI/3) {
                    robot.drivetrain.setControls(0.6,0,-0.35);
                } else {
                    robot.drivetrain.setTargetPoint(30,45,Math.PI/2);
                }

                if (time.seconds() > foundationPullTime) {
                    foundationPull = true;
                    robot.grabber.releaseFoundation();
                    depositX = robot.drivetrain.x; depositY = robot.drivetrain.y; depositTheta = robot.drivetrain.currentheading;
                    Waypoint[] skystone2PathWaypoints = new Waypoint[] {
                            new Waypoint(30, 45, Math.PI / 2, 30, 100,0, 0),
                            new Waypoint(26, skystoneY - 24, Math.PI / 3, 30, 10,0, 1.75),
                            new Waypoint(49, skystoneY - 20, Math.PI / 4 + 0.2, 30, -10,0, skystone2Time)
                    };
                    skystone2Path = new Path(new ArrayList<>(Arrays.asList(skystone2PathWaypoints)));
                    Log.w("auto", "Deposit Cor: " + depositX + " " + depositY + " " + depositTheta);
                }
            }

            // Get Second Skystone
            else if (!skystone2) {
                double currentTime = Math.min(skystone2Time, time.seconds());
                Pose robotPose = skystone2Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPoint(robotPose.getX(),robotPose.getY(),robotPose.getTheta());

                if (time.seconds() > skystone2Time + 0.5 || robot.stoneInRobot) {
                    skystone2 = true;
                }
            }
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