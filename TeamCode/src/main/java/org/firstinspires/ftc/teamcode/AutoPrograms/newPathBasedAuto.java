package org.firstinspires.ftc.teamcode.AutoPrograms;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.skyStoneDetector;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.Splines.Path;
import org.firstinspires.ftc.teamcode.Splines.Pose;
import org.firstinspires.ftc.teamcode.Splines.Spline;
import org.firstinspires.ftc.teamcode.Splines.Waypoint;
import org.firstinspires.ftc.teamcode.iLQR.Point2D;

import java.util.ArrayList;
import java.util.Arrays;

@Autonomous(name = "2 block Red Auto")
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
        robot = new Robot(this, 9, 111, 0, true,true);
        robot.logger.startLogging();
        robot.grabber.releaseFoundation();
        robot.intake.setControls(0);
        robot.stacker.unClampStone();
        robot.stacker.goHome();

        // Segment Finished Variables
        boolean skystone1 = false;
        boolean toFoundation1 = false;
        boolean foundationPull = false;
        boolean skystone2 = false;
        boolean toFoundation2 = false;
        boolean stone3 = false;
        boolean toFoundation3 = false;
        boolean stone4 = false;
        boolean toFoundation4 = false;
        boolean stone5 = false;
        boolean toFoundation5 = false;
        boolean toTape = false;

        // deposit position
        double depositX, depositY, depositTheta;

        // After Start
        waitForStart();

        // Skystone Position Variables
        int skystonePos = (int) detector.getPosition();
        double skystoneY = robot.drivetrain.y;
        if (skystonePos == 1) {
            skystoneY = 129;
        }
        else if (skystonePos == 2) {
            skystoneY = 121;
        }
        else if (skystonePos == 3) {
            skystoneY = 113;
        }

        // Path time variables
        // stone 1 times
        double skystone1Time = 1.25;
        double toFoundation1Time = 3.25;
        double foundationPullTime = 2.25;
        double skystone2Time = 2;
        double toFoundation2Time = 2.5;
        double stone3Time = 2;
        double toFoundation3Time = 2;
        double stone4Time = 2.5;
        double toFoundation4Time = 2.5;
        double stone5Time = 2.5;
        double toFoundation5Time = 2.5;

        if (skystonePos == 2) {
            skystone1Time = 1.5;
            toFoundation1Time = 3;
            skystone2Time = 2;
            toFoundation2Time = 2.5;
            stone3Time = 2;
            toFoundation3Time = 2;
            stone4Time = 2.5;
            toFoundation4Time = 2.5;
            stone5Time = 2.5;
            toFoundation5Time = 2.5;

        } else if(skystonePos == 3) {
            skystone1Time = 1.25;
            toFoundation1Time = 2.75;
            skystone2Time = 2;
            toFoundation2Time = 2.5;
            stone3Time = 2.25;
            toFoundation3Time = 2;
            stone4Time = 2.5;
            toFoundation4Time = 2.5;
            stone5Time = 2.5;
            toFoundation5Time = 2.5;
        }

        // stone 3-5 locations
        Point2D[][] stoneLocations = {
                {new Point2D(54, 100), new Point2D(74, 115), new Point2D(60, 125)},
                {new Point2D(55,114), new Point2D(55,93), new Point2D(45,130)},
                {new Point2D(55,108), new Point2D(45,126), new Point2D(45,130)}
        };

        // stop detector
        detector.setActive(false);

        // Defining paths
        Waypoint[] skystone1PathWaypoints = {
                new Waypoint(9,111,0,20,100,0,0),
                new Waypoint(45,skystoneY,Math.PI / 4 + 0.2, 20, -100, 0, skystone1Time)
        };
        Path skystone1Path = new Path(new ArrayList<>(Arrays.asList(skystone1PathWaypoints)));
        Spline skystone1ThetaSpline = new Spline(Math.PI/6, 0, 8, Math.PI / 4 + 0.2, 0, 0, skystone1Time);

        Path toFoundation1Path = null;
        Path skystone2Path = null;
        Path toFoundation2Path = null;
        Path stone3Path = null;
        Path toFoundation3Path = null;
        Path stone4Path = null;
        Path toFoundation4Path = null;
        Path stone5Path = null;
        Path toFoundation5Path = null;

        // Time Used to End Segments After a Certain Period of Time
        ElapsedTime time = new ElapsedTime();

        robot.intake.setControls(robot.intakePower);
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

                if (robot.stoneInRobot || time.seconds() > (skystone1Time + 0.5)) {
                    //setting variable to move on from this segment
                    skystone1 = true;

                    //defining the tofoundation path
                    Waypoint[] toFoundation1PathWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.currentheading, -30, -100,0, 0),
                            new Waypoint(36, skystoneY - 25, Math.PI / 3, -50, -40,0, 1),
                            new Waypoint(31, 35, Math.PI / 2, -30, -30,0, 2),
                            new Waypoint(50, 24, Math.PI, -30, 100,0, toFoundation1Time)
                    };
                    toFoundation1Path = new Path(new ArrayList<>(Arrays.asList(toFoundation1PathWaypoints)));
//                    robot.intake.setControls(0);

                    //reset time
                    time.reset();
                }
            }

            // Go to Foundation to Deposit First Skystone
            else if (!toFoundation1) {
                double currentTime = Math.min(toFoundation1Time, time.seconds());
                Pose robotPose = toFoundation1Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPoint(robotPose.getX(),robotPose.getY(),robotPose.getTheta()+Math.PI);

                if (time.seconds() > (toFoundation1Time + 1.0)) {
                    robot.grabber.grabFoundation();
                }
                if (time.seconds() > (toFoundation1Time + 1.5)) {
                    toFoundation1 = true;
                    robot.depositAuto();
                    time.reset();
                }
            }

            // Turn and Pull Foundation
            else if (!foundationPull) {
                if (robot.drivetrain.x > 40 && robot.drivetrain.currentheading < 2*Math.PI/3 ) {
                    robot.drivetrain.setControls(0.8, 0, 0);
                } else if (robot.drivetrain.currentheading > 2*Math.PI/3) {
                    robot.drivetrain.setControls(0.6,0,-0.35);
                } else {
                    robot.drivetrain.setTargetPoint(30,45,Math.PI/2);
                }

                if (time.seconds() > foundationPullTime) {
                    foundationPull = true;
                    robot.grabber.releaseFoundation();
                    depositX = robot.drivetrain.x; depositY = robot.drivetrain.y; depositTheta = robot.drivetrain.currentheading;
                    Log.w("auto", "Deposit Cor: " + depositX + " " + depositY + " " + depositTheta);

                    Waypoint[] skystone2PathWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x,robot.drivetrain.y, robot.drivetrain.currentheading, 10, 75,0, 0),
                            new Waypoint(31, skystoneY - 28, Math.PI / 3, 30, 10,-3, 1.5),
                            new Waypoint(47, skystoneY - 24, Math.PI/4, 10, -100,0, skystone2Time)
                    };
                    skystone2Path = new Path(new ArrayList<>(Arrays.asList(skystone2PathWaypoints)));
//                    robot.intake.setControls(0.7);
                    time.reset();
                }
            }

            // Get Second Skystone
            else if (!skystone2) {
                double currentTime = Math.min(skystone2Time, time.seconds());
                if (time.seconds()< (skystone2Time + 0.5)) {
                    Pose robotPose = skystone2Path.getRobotPose(currentTime);
                    robot.drivetrain.setTargetPoint(robotPose.getX(),robotPose.getY(),robotPose.getTheta());
                }
                else {
                    robot.drivetrain.setTargetPoint(50,skystoneY-18, Math.PI/4 + 0.25);
                }

                if (time.seconds() > (skystone2Time + 2) || (robot.stoneInRobot && robot.drivetrain.y>72)) {
                    skystone2 = true;
                    Waypoint[] toFoundation2PathWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.currentheading, -30, -100,0, 0),
                            new Waypoint(33, skystoneY - 32, Math.PI / 2, -50, -10,0, 1),
                            new Waypoint(33, 45, Math.PI / 2, -50, -10,0, 2),
                            new Waypoint(40, 28, Math.PI / 2, -30, 50,0, toFoundation2Time)
                    };
                    toFoundation2Path = new Path(new ArrayList<>(Arrays.asList(toFoundation2PathWaypoints)));

//                    robot.intake.setControls(0);
                    time.reset();
                }
            }

            // Go to Foundation to Deposit Second Skystone
            else if (!toFoundation2) {
                double currentTime = Math.min(toFoundation2Time, time.seconds());
                Pose robotPose = toFoundation2Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPoint(robotPose.getX(),robotPose.getY(),robotPose.getTheta()+Math.PI);

                if(robot.drivetrain.y<65){
                    robot.depositAuto();
                }
                if (time.seconds() > (toFoundation2Time + 0.5)) {
                    toFoundation2 = true;
                    Waypoint[] stone3PathWaypoints;
                    if(skystonePos == 1){
                        stone3PathWaypoints = new Waypoint[] {
                                new Waypoint(robot.drivetrain.x,robot.drivetrain.y, robot.drivetrain.currentheading, 10, 100,0, 0),
                                new Waypoint(31, stoneLocations[skystonePos-1][0].getY() - 26, Math.PI / 3, 30, 10,-3, 1.5),
                                new Waypoint(stoneLocations[skystonePos-1][0].getX(), stoneLocations[skystonePos-1][0].getY(), Math.PI/4-0.2, 10, -100,0, stone3Time)
                        };
                    }
                    else{
                        stone3PathWaypoints = new Waypoint[] {
                                new Waypoint(robot.drivetrain.x,robot.drivetrain.y, robot.drivetrain.currentheading, 10, 100,0, 0),
                                new Waypoint(31, stoneLocations[skystonePos-1][0].getY() - 26, Math.PI / 3, 30, 10,-3, 1.5),
                                new Waypoint(stoneLocations[skystonePos-1][0].getX(), stoneLocations[skystonePos-1][0].getY(), Math.PI/4, 10, -100,0, stone3Time)
                        };

                    }

                    stone3Path = new Path(new ArrayList<>(Arrays.asList(stone3PathWaypoints)));
//                    robot.intake.setControls(0.7);
                    time.reset();

                }
            }
            // Get Third stone
            else if (!stone3) {
                double currentTime = Math.min(stone3Time, time.seconds());

                Pose robotPose = stone3Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPoint(robotPose.getX(), robotPose.getY(), robotPose.getTheta());

                if (time.seconds() > (stone3Time + 1.5) || (robot.stoneInRobot && robot.drivetrain.y>72)) {
                    stone3 = true;
                    Waypoint[] toFoundation3PathWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.currentheading, -30, -100,0, 0),
                            new Waypoint(31, stoneLocations[skystonePos-1][0].getY() - 32, Math.PI / 2, -50, -10,0, 1),
                            new Waypoint(30, 37, Math.PI / 2, -30, 50,0, toFoundation3Time)
                    };
                    toFoundation3Path = new Path(new ArrayList<>(Arrays.asList(toFoundation3PathWaypoints)));
//                    robot.intake.setControls(0);
                    time.reset();
                }
            }
            //return to foundation 3
            else if(!toFoundation3){
                double currentTime = Math.min(toFoundation3Time, time.seconds());

                Pose robotPose = toFoundation3Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPoint(robotPose.getX(), robotPose.getY(), robotPose.getTheta()+Math.PI);

                if(robot.drivetrain.y<65){
                    robot.depositAuto();
                }
                if (time.seconds() > (toFoundation3Time + 0.5)) {
                    toFoundation3 = true;
                    Waypoint[] stone4PathWaypoints;
                    if (skystonePos == 2) {
                        stone4PathWaypoints = new Waypoint[]{
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.currentheading, 10, 100, 0, 0),
                                new Waypoint(31, stoneLocations[skystonePos - 1][1].getY() - 16, Math.PI / 3, 30, 10, -3, 1.5),
                                new Waypoint(stoneLocations[skystonePos - 1][1].getX(), stoneLocations[skystonePos - 1][1].getY(), Math.PI/8, 10, -100, 0, stone4Time)
                        };
                    }
                    else if (skystonePos == 3) {
                        stone4PathWaypoints = new Waypoint[]{
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.currentheading, 10, 100, 0, 0),
                                new Waypoint(31, stoneLocations[skystonePos - 1][1].getY() - 21, Math.PI / 3, 30, 10, -3, 1.5),
                                new Waypoint(stoneLocations[skystonePos - 1][1].getX(), stoneLocations[skystonePos - 1][1].getY(), Math.PI/2, 10, -100, 0, stone4Time)
                        };
                    }
                    else { //pos 1
                        stone4PathWaypoints = new Waypoint[]{
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.currentheading, 10, 100, 0, 0),
                                new Waypoint(31, stoneLocations[skystonePos - 1][1].getY() - 26, Math.PI / 3, 30, 10, -3, 1.5),
                                new Waypoint(stoneLocations[skystonePos - 1][1].getX(), stoneLocations[skystonePos - 1][1].getY(), Math.PI / 4, 10, -100, 0, stone4Time)
                        };
                    }
                    stone4Path = new Path(new ArrayList<>(Arrays.asList(stone4PathWaypoints)));
//                    robot.intake.setControls(0.7);
                    time.reset();
                }
            }
            //Get fourth stone
            else if(!stone4){
                double currentTime = Math.min(stone4Time, time.seconds());

                Pose robotPose = stone4Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPoint(robotPose.getX(), robotPose.getY(), robotPose.getTheta());

                if (time.seconds() > (stone4Time + 0.5) || (robot.stoneInRobot && robot.drivetrain.y>72)) {
                    stone4 = true;
                    Waypoint[] toFoundation4PathWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.currentheading, -30, -100,0, 0),
                            new Waypoint(31, stoneLocations[skystonePos-1][1].getY() - 17, Math.PI / 2, -50, -10,0, 1),
                            new Waypoint(30, 37, Math.PI / 2, -30, 50,0, toFoundation4Time)
                    };
                    toFoundation4Path = new Path(new ArrayList<>(Arrays.asList(toFoundation4PathWaypoints)));
//                    robot.intake.setControls(0);
                    time.reset();
                }
            }
            //return to foundation 4
            else if(!toFoundation4){
                double currentTime = Math.min(toFoundation4Time, time.seconds());

                Pose robotPose = toFoundation4Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPoint(robotPose.getX(), robotPose.getY(), robotPose.getTheta()+Math.PI);

                if(robot.drivetrain.y<55){
                    robot.depositAuto();
                }
                if (time.seconds() > (toFoundation4Time + 0.5)) {
                    toFoundation4 = true;
                    Waypoint[] stone5PathWaypoints;
                    if (skystonePos == 2 || skystonePos == 3) {
                        stone5PathWaypoints = new Waypoint[]{
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.currentheading, 10, 100, 0, 0),
                                new Waypoint(31, stoneLocations[skystonePos - 1][2].getY() - 26, Math.PI / 3, 30, 10, -3, 1.5),
                                new Waypoint(stoneLocations[skystonePos - 1][2].getX(), stoneLocations[skystonePos - 1][2].getY(), Math.PI / 2, 10, -100, 0, stone5Time)
                        };
                    }
                    else { //pos 1
                        stone5PathWaypoints = new Waypoint[]{
                                new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.currentheading, 10, 100, 0, 0),
                                new Waypoint(31, stoneLocations[skystonePos - 1][2].getY() - 26, Math.PI / 3, 30, 10, -3, 1.5),
                                new Waypoint(stoneLocations[skystonePos - 1][2].getX(), stoneLocations[skystonePos - 1][2].getY(), Math.PI / 4, 10, -100, 0, stone5Time)
                        };
                    }
                    stone5Path = new Path(new ArrayList<>(Arrays.asList(stone5PathWaypoints)));
//                    robot.intake.setControls(0.7);
                    time.reset();
                }
            }
            //Get fifth stone
            else if(!stone5){
                double currentTime = Math.min(stone5Time, time.seconds());

                Pose robotPose = stone5Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPoint(robotPose.getX(), robotPose.getY(), robotPose.getTheta());

                if (time.seconds() > (stone5Time + 0.5) || (robot.stoneInRobot && robot.drivetrain.y>72)) {
                    stone5 = true;
                    Waypoint[] toFoundation5PathWaypoints = new Waypoint[] {
                            new Waypoint(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.currentheading, -30, -100,0, 0),
                            new Waypoint(31, stoneLocations[skystonePos-1][2].getY() - 32, Math.PI / 2, -50, -10,0, 1),
                            new Waypoint(30, 37, Math.PI / 2, -30, 50,0, toFoundation5Time)
                    };
                    toFoundation5Path = new Path(new ArrayList<>(Arrays.asList(toFoundation5PathWaypoints)));
//                    robot.intake.setControls(0);
                    time.reset();
                }
            }
            //return to foundation 5
            else if(!toFoundation5){
                double currentTime = Math.min(toFoundation5Time, time.seconds());

                Pose robotPose = toFoundation5Path.getRobotPose(currentTime);
                robot.drivetrain.setTargetPoint(robotPose.getX(), robotPose.getY(), robotPose.getTheta()+Math.PI);

                if(robot.drivetrain.y<65){
                    robot.depositAuto();
                }
                if (time.seconds() > (toFoundation5Time + 0.5)) {
                    toFoundation5 = true;
                    time.reset();
                }
            }

            // Park Under Skybridge
            else if (!toTape) {
                robot.drivetrain.setTargetPoint(33, 72, Math.PI / 2);
            }
            else {
                robot.drivetrain.setControls(0,0,0);
            }

            //adding relevant information to telemetry
            robot.addPacket("Battery Voltage", robot.getBatteryVoltage());
            robot.addPacket("Time", (System.currentTimeMillis()-robot.startTime)/1000);
            robot.addPacket("Skystone Position", skystonePos);
        }

        robot.update();
        robot.logger.stopLogging();
    }

    public void log(String message) {
        Log.w("auto", message);
    }
}