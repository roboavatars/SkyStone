package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.skyStoneDetector;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.Splines.Spline;
import org.firstinspires.ftc.teamcode.Splines.SplineGenerator;

@Autonomous
public class RedAuto extends LinearOpMode {
    
    private Robot robot;
    private skyStoneDetector detector = new skyStoneDetector(this);
    
    @Override
    public void runOpMode() {
        // initialize skystone detector
        detector.initializeCamera();
        detector.start();
        detector.setActive(true);
        detector.isAllianceRed(true);
        
        // initialize robot
        robot = new Robot(this, 9, 111, 0);
        robot.grabber.releaseFoundation();
        
        // after start
        waitForStart();
        
        // skystone position variables
        double skystonePos = detector.getPosition();
        double skystoneY = robot.drivetrain.y;
        
        // segment finished variables
        boolean skystone1 = false;
        boolean backToCenter1 = false;
        boolean toFoundation1 = false;
        boolean approachFoundation = false;
        boolean pullFoundation = false;
        boolean turnFoundation = false;
        boolean pushFoundation = false;
        boolean toQuarry = false;
        boolean skystone2 = false;
        boolean backToCenter2 = false;
        boolean toFoundation2 = false;
        boolean toTape = false;
        
        // spline time variables
        double skystone1Time = 2.5;
        double backToCenterTime = 1;
        double toFoundation1Time = 1.75;
        double toQuarryTime = 2;
        double skystone2Time = 2;
        
        // set skystone y coordinate according to skystone position
        if (skystonePos == 1) {
            skystoneY = 132;
        } else if (skystonePos == 2) {
            skystoneY = 123;
        } else if (skystonePos == 3) {
            skystoneY = 114;
        }
        
        // generate splines
        SplineGenerator splineGenerator = new SplineGenerator();
        Spline[] skystone1Spline = splineGenerator.SplineBetweenTwoPoints(9, 111,
                45, skystoneY, 0, Math.PI / 4, 0, 0,
                20, 0, 0, 0, skystone1Time);
        
        Spline[] backToCenterSpline = splineGenerator.SplineBetweenTwoPoints(45, skystoneY,
                36, skystoneY - 12, Math.PI / 4, Math.PI / 2, 0, -20,
                -20, -20, 0, 0, backToCenterTime);
        Spline backToCenterThetaSpline = new Spline(Math.PI / 4, 0, 0, Math.PI / 2, 0, 0, backToCenterTime);
        
        Spline[] toFoundationSpline = splineGenerator.SplineBetweenTwoPoints(36, skystoneY - 12,
                36, 25, Math.PI / 2, Math.PI, 0, 0,
                -20, 0, 0, 0, toFoundation1Time);
        Spline toFoundationThetaSpline = new Spline(Math.PI / 2, 0, 0, Math.PI, 0, 0, toFoundation1Time);
    
        Spline[] toQuarrySpline = splineGenerator.SplineBetweenTwoPoints(35, 29,
                24, skystoneY - 35, Math.PI / 2, Math.PI / 4, 0, 0,
                20, 0, 0, 0, toQuarryTime);
        Spline toQuarryThetaSpline = new Spline(Math.PI / 2, 0, 0, Math.PI / 4, 0, 0, toQuarryTime);
    
        Spline[] skystone2Spline = splineGenerator.SplineBetweenTwoPoints(36, 72,
                45, skystoneY - 24, Math.PI / 2, Math.PI / 4, 30, 0,
                20, 0, 0, 0, skystone2Time);
        
        // time used to end segments after a certain period of time
        ElapsedTime time = new ElapsedTime();
        
        // robot move loop
        while (opModeIsActive()) {
            
            // update robot's coordinates on field from odometry pods
            robot.update();
            
            // get the first skystone
            if (!skystone1) {
                robot.intake.setControls(1);
                
                double currentTime = Math.min(2, time.seconds());
                robot.drivetrain.setTargetPoint(skystone1Spline[0].position(currentTime), skystone1Spline[1].position(currentTime),
                        Math.PI / 4);
                if (time.seconds() > skystone1Time + 1) {
                    skystone1 = true;
                    robot.intake.setControls(0);
                    detector.setActive(false);
                    detector.interrupt();
                    backToCenterSpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            36, skystoneY - 12, robot.drivetrain.currentheading, Math.PI / 2, 0, -70,
                            -20, -50, 0, 0, backToCenterTime);
                    backToCenterThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI / 2, 0, 0, backToCenterTime);
                    time.reset();
                }
            }
            
            // go to the center of the tile closet to the neutral skybridge to avoid hitting alliance partner's robot
            else if (!backToCenter1) {
                double currentTime = Math.min(backToCenterTime, time.seconds());
                robot.drivetrain.setTargetPoint(backToCenterSpline[0].position(currentTime), backToCenterSpline[1].position(currentTime),
                        backToCenterThetaSpline.position(currentTime));
                if (time.seconds() > backToCenterTime) {
                    backToCenter1 = true;
                    toFoundationSpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            31, 36, robot.drivetrain.currentheading, Math.PI, -70, 0,
                            -50, 0, 0, 0, toFoundation1Time);
                    toFoundationThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI, 0, 0, toFoundation1Time);
                    time.reset();
                }
            }
            
            // get near the foundation
            else if (!toFoundation1) {
                double currentTime = Math.min(toFoundation1Time, time.seconds());
                robot.drivetrain.setTargetPoint(toFoundationSpline[0].position(currentTime), toFoundationSpline[1].position(currentTime),
                        toFoundationThetaSpline.position(currentTime));
                if (time.seconds() > toFoundation1Time + 1) {
                    toFoundation1 = true;
                    time.reset();
                }
            }
            
            // approach and align robot with foundation and grab it
            else if (!approachFoundation) {
                robot.drivetrain.setTargetPoint(44, 25, Math.PI);
                if (time.seconds() > 0.5) {
                    robot.grabber.grabFoundation();
                }
                if (time.seconds() > 1.5) {
                    approachFoundation = true;
                    time.reset();
                }
                
            }
            
            // pull the foundation so that it is in front of the building site
            else if (!pullFoundation) {
                robot.drivetrain.setTargetPoint(26, robot.drivetrain.y, Math.PI, 0.8, 0, 0.8);
                if (time.seconds() >  1) {
                    pullFoundation = true;
                    time.reset();
                }
            }
            
            // turn the foundation 90 degrees
            else if (!turnFoundation) {
                robot.drivetrain.setTargetPoint(35, 35, Math.PI / 2, 0, 0, 3);
                if (time.seconds() > 1) {
                    turnFoundation = true;
                    time.reset();
                }
            }
            
            // push the foundation forward to score it in building zone, unclamp it
            // ADD CODE- deposit skystone
            else if (!pushFoundation) {
                robot.drivetrain.setTargetPoint(35, 29, Math.PI / 2, 0.1, 0.4, 0.8);
                if (time.seconds() > 1) {
                    pushFoundation = true;
                    robot.grabber.releaseFoundation();
                    toQuarrySpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            24, skystoneY - 30, robot.drivetrain.currentheading, Math.PI / 4, 0, 0,
                            20, 0, 0, 0, toQuarryTime);
                    toQuarryThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI / 4, 0, 0, toQuarryTime);
                    time.reset();
                }
            }
            
            // travel back to the quarry to get second skystone
            else if (!toQuarry) {
                double currentTime = Math.min(toQuarryTime, time.seconds());
                robot.drivetrain.setTargetPoint(toQuarrySpline[0].position(currentTime), toQuarrySpline[1].position(currentTime),
                        toQuarryThetaSpline.position(currentTime));
                if (time.seconds() > toQuarryTime) {
                    toQuarry = true;
                    robot.intake.setControls(1);
                    skystone2Spline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            45, skystoneY - 25, robot.drivetrain.currentheading, Math.PI / 4, 30, 0,
                            20, 0, 0, 0, skystone2Time);
                    time.reset();
                }
            }
            
            // get the second skystone
            else if (!skystone2) {
                double currentTime = Math.min(skystone2Time, time.seconds());
                robot.drivetrain.setTargetPoint(skystone2Spline[0].position(currentTime), skystone2Spline[1].position(currentTime),
                        Math.PI / 4);
                if (time.seconds() > skystone2Time + 1) {
                    robot.intake.setControls(0);
                    skystone2 = true;
                    time.reset();
                }
            }

            else if (!backToCenter2) {
                robot.drivetrain.setTargetPoint(35, 85, Math.PI / 2, 0.2, 0.2, 0.8);
                if (time.seconds() > 1) {
                    backToCenter2 = true;
                    time.reset();
                }
            }
            
            // go to foundation to score second skystone
            // ADD CODE- deposit skystone and make sure it does this in different location than first one
            else if (!toFoundation2) {
                robot.drivetrain.setTargetPoint(35, 40, Math.PI / 2, 0.2, 0.03, 0.8);
                if (time.seconds() > 3.5) {
                    toFoundation2 = true;
                    time.reset();
                }
            }
            
            // park at tape under the alliance skybridge
            else if (!toTape) {
                robot.drivetrain.setTargetPoint(30, 67, Math.PI / 2, 0.2, 0.2, 0.8);
                if (time.seconds() > 1) {
                    toTape = true;
                    time.reset();
                }
            }
            
            // stop robot
            else {
                robot.drivetrain.setControls(0, 0, 0);
            }
            
            telemetry.addData("skystone position", skystonePos);
            telemetry.addData("x", robot.drivetrain.x);
            telemetry.addData("y", robot.drivetrain.y);
            telemetry.addData("theta", robot.drivetrain.currentheading);
            telemetry.update();
        }
    }
}