//package org.firstinspires.ftc.teamcode.AutoPrograms;
//
//import android.util.Log;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.OpenCV.skyStoneDetector;
//import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
//import org.firstinspires.ftc.teamcode.Splines.Spline;
//import org.firstinspires.ftc.teamcode.Splines.SplineGenerator;
//
//@Autonomous @Disabled
//public class BlueAuto extends LinearOpMode {
//
//    /*
//    *
//    * please update
//    *
//    *
//    *
//    *
//    *
//    *
//    * */
//
//
//
//    private Robot robot;
//    private skyStoneDetector detector = new skyStoneDetector(this);
//
//    @Override
//    public void runOpMode() {
//
//        // initialize skystone detector
//        detector.initializeCamera();
//        detector.start();
//        detector.setActive(true);
//        detector.isAllianceRed(false);
//
//        // initialize robot
//        robot = new Robot(this, 135, 111, Math.PI, false);
//        robot.logger.startLogging();
//        robot.grabber.releaseFoundation();
//        robot.intake.setControls(0);
//        robot.stacker.unClampStone();
//        robot.stacker.goHome();
//
//        // after start
//        waitForStart();
//
//        // push foundation coordinates
//        double depositX = 0, depositY = 0, depositTheta = 0;
//
//        // skystone position variables
//        double skystonePos = detector.getPosition();
//        double skystoneY = robot.drivetrain.y;
//
//        // segment finished variables
//        boolean skystone1 = false;
//        boolean backToCenter1 = false;
//        boolean toFoundation1 = false;
//        boolean foundationTurn = false;
//        boolean approachFoundation = false;
//        boolean pullFoundation = false;
//        boolean turnFoundation = false;
//        boolean pushFoundation = false;
//        boolean toQuarry1 = false;
//        boolean skystone2 = false;
//        boolean backToCenter2 = false;
//        boolean toFoundation2 = false;
//        boolean toTape = false;
//
//        // spline time variables
//        double skystone1Time = 2.5;
//        double backToCenterTime = 0.75;
//        double foundationTurnTime = 1.5;
//        double toQuarryTime = 2;
//        double skystone2Time = 2;
//
//        // set skystone y coordinate according to skystone position
//        if (skystonePos == 1) skystoneY = 129;
//        else if (skystonePos == 2) skystoneY = 121;
//        else if (skystonePos == 3) skystoneY = 112;
//
//        // generate splines
//        SplineGenerator splineGenerator = new SplineGenerator();
//        Spline[] skystone1Spline = splineGenerator.SplineBetweenTwoPoints(135, 111,
//                99, skystoneY, Math.PI, 3 * Math.PI / 4, 0, 0,
//                20, 0, 0, 0, skystone1Time);
//
//        Spline[] backToCenterSpline = splineGenerator.SplineBetweenTwoPoints(99, skystoneY,
//                114, skystoneY - 12, 3 * Math.PI / 4, Math.PI / 2, 0, -70,
//                -20, -50, 0, 0, backToCenterTime);
//        Spline backToCenterThetaSpline = new Spline(3 * Math.PI / 4, 0, 0, Math.PI / 2, 0, 0, backToCenterTime);
//
//        Spline[] toQuarrySpline = splineGenerator.SplineBetweenTwoPoints(114, 29,
//                120, skystoneY - 30, Math.PI / 2, 3 * Math.PI / 4, 0, 0,
//                20, 0, 0, 0, toQuarryTime);
//        Spline toQuarryThetaSpline = new Spline(Math.PI / 2, 0, 0, 3 * Math.PI / 4, 0, 0, toQuarryTime);
//
//        Spline[] skystone2Spline = splineGenerator.SplineBetweenTwoPoints(120, skystoneY - 30,
//                99, skystoneY - 26, Math.PI / 2, 3 * Math.PI / 4, 30, 0,
//                20, 0, 0, 0, skystone2Time);
//
//        // time used to end segments after a certain period of time
//        ElapsedTime time = new ElapsedTime();
//
//        double lastime = 0;
//        robot.intake.setControls(0.6);
//        sleep(33);
//
//        log("ss1");
//
//        // robot move loop
//        while (opModeIsActive()) {
//
//            // update robot's coordinates on field from odometry pods
//            robot.update();
//
//            // get the first skystone
//            if (!skystone1) {
//                double currentTime = Math.min(2, time.seconds());
//
//                // if less than moving time, continue moving
//                if (time.seconds() < skystone1Time) {
//                    robot.drivetrain.setTargetPoint(skystone1Spline[0].position(currentTime), skystone1Spline[1].position(currentTime),
//                            3 * Math.PI / 4 - 0.15);
//                }
//                // if skystone is clamped or robot has been trying to intake stone for too long, move on
//                // end segment, disable ss detector, recalc splines, reset time
//                else if (robot.stacker.stoneClamped || time.seconds() > skystone1Time + 3) {
//                    skystone1 = true;
//                    detector.setActive(false);
//                    backToCenterSpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
//                            114, skystoneY - 15, robot.drivetrain.currentheading, Math.PI / 2, 0, -70,
//                            -20, -50, 0, 0, backToCenterTime);
//                    backToCenterThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI / 2, 0, 0, backToCenterTime);
//                    time.reset(); log("backcenter1");
//                }
//                // if skystone has not been clamped, adjust position to try to suck it in
//                else {
//                    log("adjusting");
//                    robot.drivetrain.setTargetPoint(robot.drivetrain.x - 1.4, robot.drivetrain.y + 1.1, robot.drivetrain.currentheading - 0.14);
//                }
//            }
//
//            // go to the center of the tile closet to the neutral skybridge to avoid hitting alliance partner's robot
//            else if (!backToCenter1) {
//                // if less than moving time, continue moving
//                double currentTime = Math.min(backToCenterTime, time.seconds());
//                robot.drivetrain.setTargetPoint(backToCenterSpline[0].position(currentTime), backToCenterSpline[1].position(currentTime),
//                        backToCenterThetaSpline.position(currentTime));
//                robot.intakeManual = true;
//                robot.intake.setControls(-1);
//
//                // if more than allotted time, end segment, reset time
//                if (time.seconds() > backToCenterTime) {
//                    backToCenter1 = true;
//                    robot.intakeManual = false;
//                    time.reset(); log("tofound1");
//                }
//            }
//
//            // get near the foundation
//            else if (!toFoundation1) {
//                // if not at position continue moving
//                robot.drivetrain.setTargetPoint(108, 55, Math.PI / 2);
//
//                // if at position, end segment, recalc splines, reset time
//                if (robot.drivetrain.y < 58) {
//                    toFoundation1 = true;
//                    time.reset(); log("foundturn");
//                }
//            }
//
//            // turn robot to align with foundation
//            else if (!foundationTurn) {
//                // if less than moving time or not at position, continue moving
//                robot.drivetrain.setTargetPoint(106,33,0);
//
//                // if at position or time met, end segment, reset time
//                if (time.seconds() > foundationTurnTime || robot.drivetrain.isAtPoseAuto(38, 33, Math.PI)) {
//                    foundationTurn = true;
//                    // if stone clamped, deposit it
//                    if (robot.stacker.stoneClamped) {
//                        robot.depositAuto();
//                    }
//                    robot.grabber.grabFoundation();
//
//                    time.reset(); log("pullfound");
//                }
//            }
//
//            // approach and align robot with foundation
//            else if (!approachFoundation) {
//                robot.drivetrain.setTargetPoint(100, 23.5, 0);
//
//                // extend arm with skystone over the foundation
//                if (robot.drivetrain.isAtPoseAuto(100, 23.5, 0) || time.seconds() > 2.5) {
//                    approachFoundation = true;
//                    time.reset();
//                }
//            }
//
//            // pull the foundation so that it is in front of the building site
//            else if (!pullFoundation) {
//                // if less than moving time or not at position, continue moving
//                robot.drivetrain.setTargetPoint(118, 30, 0, 0.6, 0.6, 0.4);
//
//                // if at position or time met, end segment, reset time
//                if (robot.drivetrain.isAtPoseAuto(118, 30, 0) || time.seconds() > 1.5) {
//                    pullFoundation = true;
//                    time.reset(); log("turnfound");
//                }
//            }
//
//            // turn the foundation 90 degrees
//            else if (!turnFoundation) {
//                // if less than moving time or not at position, continue moving
//                robot.drivetrain.setTargetPoint(109, 35, Math.PI / 2, 0, 0, 2.75);
//
//                // if at position or time met, end segment, reset time
//                if (robot.drivetrain.isAtPoseAuto(109, 35, Math.PI / 2, 144, 144, 0.1) || time.seconds() > 1) {
//                    turnFoundation = true;
//                    time.reset(); log("pushfound");
//                }
//            }
//
//            // push the foundation forward to score it in building zone
//            else if (!pushFoundation) {
//                // if arm is home or less than moving time, continue moving
//                robot.drivetrain.setTargetPoint(109, 28, Math.PI / 2, 0.2, 0.2, 1.5);
//
//                // if arm is home or time met, end segment, release foundation, recalc splines, reset time
//                if (robot.stacker.isArmHome() || time.seconds() > 2) {
//                    depositX = robot.drivetrain.x; depositY = robot.drivetrain.y; depositTheta = robot.drivetrain.currentheading;
//                    Log.w("auto", "Deposit Cor: " + depositX + " " + depositY + " " + depositTheta);
//
//                    pushFoundation = true;
//                    robot.grabber.releaseFoundation();
//                    toQuarrySpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
//                            117.5, skystoneY - 30, robot.drivetrain.currentheading, Math.PI - Math.PI/2.5, 0, 0,
//                            20, 0, 0, 0, toQuarryTime);
//                    toQuarryThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI - Math.PI/2.5, 0, 0, toQuarryTime);
//                    time.reset(); log("toquarry1");
//                }
//            }
//
//            // travel back to the quarry to get second skystone
//            else if (!toQuarry1) {
//                // if less than moving time, continue moving
//                double currentTime = Math.min(toQuarryTime, time.seconds());
//                robot.drivetrain.setTargetPoint(toQuarrySpline[0].position(currentTime), toQuarrySpline[1].position(currentTime),
//                        toQuarryThetaSpline.position(currentTime), 0.2,0.2,1.2);
//
//                // if time met, end segment, recalc splines, reset time
//                if (time.seconds() > toQuarryTime + 1 || robot.drivetrain.y>(skystoneY - 30)) {
//                    toQuarry1 = true;
//                    skystone2Spline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
//                            99, skystoneY - 26, robot.drivetrain.currentheading, 3 * Math.PI / 4, 30, 0,
//                            20, 0, 0, 0, skystone2Time);
//                    time.reset(); log("ss2");
//                }
//            }
//
//            // get the second skystone
//            else if (!skystone2) {
//                double currentTime = Math.min(skystone2Time, time.seconds());
//
//                // if not at quarry row, continue moving
//                if (time.seconds() < skystone2Time) {
//                    robot.drivetrain.setTargetPoint(skystone2Spline[0].position(currentTime), skystone2Spline[1].position(currentTime),
//                            3 * Math.PI / 4);
//                }
//                // if skystone is clamped or robot has been trying to intake stone for too long, move on
//                // end segment, reset time
//                else if (robot.stacker.stoneClamped || time.seconds() > skystone2Time + 3) {
//                    skystone2 = true;
//                    time.reset(); log("backcenter2");
//                }
//                // go to tape if skystone not collected, end current, backtocenter2, and tofoundation2 segments, reset time
//                else if (!robot.stoneInRobot && time.seconds() > skystone2Time + 3) {
//                    skystone2 = true;
//                    backToCenter2 = true;
//                    toFoundation2 = true;
//                    time.reset(); log("totapeshort");
//                }
//                // if skystone has not been clamped, adjust position to try to suck it in
//                else {
//                    log("adjusting");
//                    robot.drivetrain.setTargetPoint(robot.drivetrain.x - 2, robot.drivetrain.y + 1.6, robot.drivetrain.currentheading - 0.14);
//                }
//            }
//
//            // go to the center of the tile closet to the neutral skybridge to avoid hitting alliance partner's robot
//            else if (!backToCenter2) {
//                // if robot position is greater than 95, continue moving
//                robot.drivetrain.setTargetPoint(111, 88, Math.PI / 2, 0.2, 0.2, 0.8);
//                robot.grabber.extendRangeSensor();
//                robot.intakeManual = true;
//                robot.intake.setControls(-1);
//
//                // if robot position is less than 95, end segment, reset time
//                if (robot.drivetrain.y < 95) {
//                    backToCenter2 = true;
//                    robot.intakeManual = false;
//                    time.reset(); log("tofound2");
//                }
//            }
//
//            // go to foundation to deposit second skystone
//            else if (!toFoundation2) {
//                // if not at position, continue moving
//                if(!robot.isAutoAlign){
//                    robot.drivetrain.setTargetPoint(depositX+6, depositY-2, depositTheta);
//                }
//
//                // if robot is at foundation, deposit stone
//                if (robot.drivetrain.y<40 && Math.abs((robot.drivetrain.y - robot.drivetrain.lasty)/(time.seconds()-lastime))<0.1) {
//                    if(robot.stoneInRobot){
//                        robot.depositAuto();
//                    }
//                    toFoundation2 = true;
//                    time.reset(); log("totapereg");
//                }
//                lastime = time.seconds();
//            }
//
//            // park at tape under the alliance skybridge
//            else if (!toTape) {
//                if(robot.stacker.isArmHome() && !robot.stacker.stoneClamped){
//                    robot.drivetrain.setTargetPoint(112, 72, Math.PI / 2, 0.2, 0.2, 0.8);
//                }
//                else if(time.seconds()>3){
//                    robot.letGo = true;
//                }
//            }
//
//            telemetry.addData("skystone position", skystonePos);
//            telemetry.addData("x", robot.drivetrain.x);
//            telemetry.addData("y", robot.drivetrain.y);
//            telemetry.addData("theta", robot.drivetrain.currentheading);
//            telemetry.update();
//        }
//
//        robot.update();
//        robot.logger.stopLogging();
//        detector.interrupt();
//    }
//
//    public void log(String message) {
//        Log.w("auto", " ");
//        Log.w("auto", message + " -------------------------------------------------------------------------");
//    }
//}