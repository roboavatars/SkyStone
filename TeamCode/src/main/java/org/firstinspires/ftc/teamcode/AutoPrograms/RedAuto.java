package org.firstinspires.ftc.teamcode.AutoPrograms;

import android.util.Log;

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
        robot = new Robot(this, 9, 111, 0, true);
        robot.logger.startLogging();
        robot.grabber.releaseFoundation();
        robot.intake.setControls(0);
        robot.stacker.unClampStone();
        robot.stacker.goHome();

        // after start
        waitForStart();

        // push foundation coordinates
        double depositX = 0, depositY = 0, depositTheta = 0;

        // skystone position variables
        double skystonePos = detector.getPosition();
        double skystoneY = robot.drivetrain.y;

        // segment finished variables
        boolean skystone1 = false;
        boolean backToCenter1 = false;
        boolean toFoundation1 = false;
        boolean foundationTurn = false;
        boolean approachFoundation = false;
        boolean foundationPull = false;
        boolean toQuarry1 = false;
        boolean skystone2 = false;
        boolean backToCenter2 = false;
        boolean toFoundation2 = false;
        boolean toQuarryStone3_5 = false;
        boolean skystoneStone3_5 = false;
        boolean backToCenterStone3_5 = false;
        boolean toFoundationStone3_5 = false;
        boolean scoreFoundation = false;
        boolean toTape = false;

        int stoneCounter = 0;

        // spline time variables
        double skystone1Time = 2;
        double backToCenterTime = 0.75;
        double foundationPullTime = 2.5;
        double toQuarryTime = 2.25;
        double skystone2Time = 2;
        double stone3_5Time = 1;

        // set skystone y coordinate according to skystone position
        if (skystonePos == 1) skystoneY = 129;
        else if (skystonePos == 2) skystoneY = 121;
        else if (skystonePos == 3) skystoneY = 112;
        detector.setActive(false);

        // generate splines
        SplineGenerator splineGenerator = new SplineGenerator();
        Spline[] skystone1Spline = splineGenerator.SplineBetweenTwoPoints(9, 111,
                45, skystoneY, 0, Math.PI / 4 + 0.2, 0, 0,
                100, -100, 0, 0, skystone1Time);
        Spline skystone1ThetaSpline = new Spline(0, 0, 8, Math.PI / 4 + 0.2, 0, 0, skystone1Time);

        Spline[] backToCenterSpline = splineGenerator.SplineBetweenTwoPoints(45, skystoneY,
                30, skystoneY - 12, Math.PI / 4, Math.PI / 2, 0, -70,
                -50, -100, 0, 0, backToCenterTime);
        Spline backToCenterThetaSpline = new Spline(Math.PI / 4, 0, 0, Math.PI / 2, 0, 0, backToCenterTime);

        Spline[] foundationPullSpline = splineGenerator.SplineBetweenTwoPoints(44, 24,
                30, 45, Math.PI, Math.PI / 2, 10, 100,
                15, 100, 0, 0, foundationPullTime);
        Spline foundationPullThetaSpline = new Spline(Math.PI, 0, 0, Math.PI / 2, 0, 0, foundationPullTime);

        Spline[] toQuarrySpline = splineGenerator.SplineBetweenTwoPoints(35 , 55,
                24, skystoneY - 30, Math.PI / 2, Math.PI / 4, -70, 0,
                -20, 0, 0, 0, toQuarryTime);
        Spline toQuarryThetaSpline = new Spline(Math.PI / 2, 0, 0, Math.PI / 4, 0, 0, toQuarryTime);

        Spline[] skystone2Spline = splineGenerator.SplineBetweenTwoPoints(24, skystoneY - 30,
                45, skystoneY - 26, Math.PI / 2, Math.PI / 4, 50, 50,
                75, 75, 0, 0, skystone2Time);
        Spline skystone2ThetaSpline = new Spline(0, 0, 8, Math.PI / 4 + 0.2, 0, 0, skystone2Time);

        Spline[] stone3_5Spline = splineGenerator.SplineBetweenTwoPoints(24, skystoneY - 30,
                45, skystoneY - 26, Math.PI / 2, Math.PI / 4, 50, 50,
                75, 75, 0, 0, stone3_5Time);
        Spline stone3_5ThetaSpline = new Spline(0, 0, 8, Math.PI / 4 + 0.2, 0, 0, stone3_5Time);

        // time used to end segments after a certain period of time
        ElapsedTime time = new ElapsedTime();

        double lastime = 0;
        robot.intake.setControls(0.6);
        sleep(33);

        log("ss1");

        // robot move loop
        while (opModeIsActive()) {

            // update robot's location and states
            robot.update();

            // get the first skystone
            if (!skystone1) {
                double currentTime = Math.min(skystone1Time, time.seconds());

                // if less than moving time, continue moving
                if (time.seconds() < skystone1Time) {
                    robot.drivetrain.setTargetPointAuto(skystone1Spline[0].position(currentTime), skystone1Spline[1].position(currentTime),
                            skystone1ThetaSpline.position(currentTime));
                }
                // if skystone is clamped or robot has been trying to intake stone for too long, move on
                else if (robot.stoneInRobot || time.seconds() > skystone1Time + 2) {
                    skystone1 = true;
                    backToCenterSpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            30, skystoneY - 15.5, robot.drivetrain.currentheading, Math.PI / 2, 0, -70,
                            -20, -50, 0, 0, backToCenterTime);
                    backToCenterThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI / 2, 0, 0, backToCenterTime);
                    time.reset(); log("backcenter1");
                }
                // if skystone has not been clamped, adjust position to try to suck it in
                else {
                    log("adjusting");
                    robot.drivetrain.setTargetPointAuto(robot.drivetrain.x + 1.4, robot.drivetrain.y + 1.1, robot.drivetrain.currentheading + 0.14);
                }
            }

            // go to the center of the tile closest to the neutral skybridge to avoid hitting alliance partner's robot
            else if (!backToCenter1) {
                double currentTime = Math.min(backToCenterTime, time.seconds());
                robot.drivetrain.setTargetPointAuto(backToCenterSpline[0].position(currentTime), backToCenterSpline[1].position(currentTime),
                        backToCenterThetaSpline.position(currentTime));

                // if more than allotted time, end segment
                if (time.seconds() > backToCenterTime) {
                    backToCenter1 = true;
                    time.reset(); log("tofound1");
                }
            }

            // get near the foundation
            else if (!toFoundation1) {
                robot.drivetrain.setTargetPointAuto(36, 55, Math.PI / 2);

                // if robot position is less than position, end segment
                if (robot.drivetrain.y < 58) {
                    toFoundation1 = true;
                    time.reset(); log("foundturn");
                }
            }

            // turn robot to align with foundation
            else if (!foundationTurn) {
                robot.drivetrain.setTargetPointAuto(42,18, Math.PI);

                // if at position or time met, end segment
                if (time.seconds() > 1 || robot.drivetrain.isAtPoseAuto(42, 18, Math.PI)) {
                    foundationTurn = true;
                    // if stone clamped, deposit it
//                    if (robot.stacker.stoneClamped) {
//                        robot.depositAuto();
//                    }
                    robot.grabber.grabFoundation();

                    time.reset(); log("appfound");
                }
            }

            // approach and align robot with foundation
            else if (!approachFoundation) {
                robot.drivetrain.setTargetPointAuto(47, 18, Math.PI);

                // if at position or time met, end segment
                if (robot.drivetrain.isAtPoseAuto(47, 18, Math.PI) && time.seconds() > 0.75) {
                    approachFoundation = true;
                    foundationPullSpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            30, 45, robot.drivetrain.currentheading, Math.PI / 2, 50, 50,
                            100, 100, 0, 0, foundationPullTime);
                    foundationPullThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI / 2, 0, 0, foundationPullTime);
                    time.reset(); log("bringfound");
                }
            }

            // turn and pull foundation
            else if (!foundationPull) {
                //double currentTime = Math.min(foundationPullTime, time.seconds());
                if (robot.drivetrain.currentheading > 2*Math.PI/3) {
                    robot.drivetrain.setControls(0,0.6,-0.35);
                }
                else {
                    robot.drivetrain.setTargetPoint(30,45,Math.PI/2);
                    //robot.drivetrain.setTargetPointAuto(foundationPullSpline[0].position(currentTime), foundationPullSpline[1].position(currentTime),
                      //      foundationPullThetaSpline.position(currentTime)/*, 0.2, 0.2, 0.8*/);
                }

                // if arm is home or time met, release foundation, end segment
                if (time.seconds() > foundationPullTime) {
                    depositX = robot.drivetrain.x; depositY = robot.drivetrain.y; depositTheta = robot.drivetrain.currentheading;
                    Log.w("auto", "Deposit Cor: " + depositX + " " + depositY + " " + depositTheta);
                    foundationPull = true;

                    robot.grabber.releaseFoundation();
//                    toQuarrySpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
//                            26.5, skystoneY - 30, robot.drivetrain.currentheading, Math.PI / 2.5, 0, 0,
//                            20, 0, 0, 0, toQuarryTime);
//                    toQuarryThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI / 2.5, 0, 0, toQuarryTime);
                    time.reset(); log("toquarry1");
                }
            }

            // travel back to the quarry to get second skystone
            else if (!toQuarry1) {
//                double currentTime = Math.min(toQuarryTime, time.seconds());
//                robot.drivetrain.setTargetPointAuto(toQuarrySpline[0].position(currentTime), toQuarrySpline[1].position(currentTime),
//                        toQuarryThetaSpline.position(currentTime), 0.2,0.2,1.2);
                robot.drivetrain.setTargetPointAuto(32, skystoneY-31.5, Math.PI/4,0.2,0.2,1.2);

                // if at position or time met, end segment
                if (time.seconds() > toQuarryTime || robot.drivetrain.y > (skystoneY-31)) {
                    toQuarry1 = true;
                    skystone2Spline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            45, skystoneY - 26, robot.drivetrain.currentheading, Math.PI / 4, 30, 0,
                            20, 0, 0, 0, skystone2Time);
                    skystone2ThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 8, Math.PI / 4 + 0.2, 0, 0, skystone2Time);
                    time.reset(); log("ss2");
                }
            }

            // get the second skystone
            else if (!skystone2) {
                double currentTime = Math.min(skystone2Time, time.seconds());

                // if not at quarry row, continue moving
                if (time.seconds() < skystone2Time) {
                    robot.drivetrain.setTargetPointAuto(skystone2Spline[0].position(currentTime), skystone2Spline[1].position(currentTime),
                            skystone2ThetaSpline.position(currentTime));
                }
                // if skystone is clamped or robot has been trying to intake stone for too long, move on
                else if (robot.stoneInRobot || time.seconds() > skystone2Time + 2) {
                    skystone2 = true;
                    time.reset(); log("backcenter2");
                }
                // if skystone has not been clamped, adjust position to try to suck it in
                else {
                    log("adjusting");
                    robot.drivetrain.setTargetPointAuto(robot.drivetrain.x + 2, robot.drivetrain.y + 1.6, robot.drivetrain.currentheading + 0.14);
                }
            }

            // go to the center of the tile closet to the neutral skybridge to avoid hitting alliance partner's robot
            else if (!backToCenter2) {
                robot.drivetrain.setTargetPointAuto(29, 88, Math.PI / 2, 0.2, 0.2, 0.8);
                //robot.grabber.extendRangeSensor();

                // if robot position is less than position, end segment
                if (robot.drivetrain.y < 88) {
                    backToCenter2 = true;
                    time.reset(); log("tofound2");
                }
            }

            // go to foundation to deposit second skystone
            else if (!toFoundation2) {
//                if(!robot.isAutoAlign){
//                    robot.drivetrain.setTargetPointAuto(depositX-6, depositY-2, depositTheta);
//                }
                robot.drivetrain.setTargetPointAuto(depositX, depositY, depositTheta);

                // if robot is at foundation, deposit stone, end segment
                if (robot.drivetrain.isAtPoseAuto(depositX,depositY,depositTheta) || time.seconds() > 2 /*&& Math.abs((robot.drivetrain.y - robot.drivetrain.lasty)/(time.seconds()-lastime))<0.1*/) {
//                    if (robot.stoneInRobot){
//                        robot.depositAuto();
//                    }
                    toFoundation2 = true;
                    time.reset(); log("toquarry");
                }
                //lastime = time.seconds();
            }

            // travel back to the quarry to get stone
            else if (!toQuarryStone3_5) {
//                double currentTime = Math.min(toQuarryTime, time.seconds());
//                robot.drivetrain.setTargetPointAuto(toQuarrySpline[0].position(currentTime), toQuarrySpline[1].position(currentTime),
//                        toQuarryThetaSpline.position(currentTime), 0.2,0.2,1.2);
                robot.drivetrain.setTargetPointAuto(31, skystoneY-31, Math.PI/4,0.2,0.2,1.2);

                // if at position or time met, end segment
                if (time.seconds() > toQuarryTime || robot.drivetrain.y > (skystoneY-31)) {
                    toQuarryStone3_5 = true;
                    stone3_5Spline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            30, skystoneY - 26, robot.drivetrain.currentheading, Math.PI / 4, 30, 0,
                            20, 0, 0, 0, skystone2Time);
                    stone3_5ThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 8, Math.PI / 4 + 0.2, 0, 0, skystone2Time);
                    time.reset(); log("getstone");
                }
            }

            // get stone
            else if (!skystoneStone3_5) {
                double currentTime = Math.min(stone3_5Time, time.seconds());

                // if not at quarry row, continue moving
                if (time.seconds() < stone3_5Time) {
                    robot.drivetrain.setTargetPointAuto(stone3_5Spline[0].position(currentTime), stone3_5Spline[1].position(currentTime),
                            stone3_5ThetaSpline.position(currentTime));
                }
                // if skystone is clamped or robot has been trying to intake stone for too long, move on
                else if (robot.drivetrain.y > 80) {
                    skystoneStone3_5 = true;
                    time.reset(); log("backcenter");
                }
//                // if skystone has not been clamped, adjust position to try to suck it in
//                else {
//                    log("adjusting");
//                    robot.drivetrain.setTargetPointAuto(robot.drivetrain.x + 2, robot.drivetrain.y + 1.6, robot.drivetrain.currentheading + 0.14);
//                }
            }

            // go to the center of the tile closet to the neutral skybridge to avoid hitting alliance partner's robot
            else if (!backToCenterStone3_5) {
                robot.drivetrain.setTargetPointAuto(29, 88, Math.PI / 2, 0.2, 0.2, 0.8);
                //robot.grabber.extendRangeSensor();

                // if robot position is less than position, end segment
                if (robot.drivetrain.y < 88) {
                    backToCenterStone3_5 = true;
                    time.reset(); log("tofound");
                }
            }

            // go to foundation to deposit stone
            else if (!toFoundationStone3_5) {
//                if(!robot.isAutoAlign){
//                    robot.drivetrain.setTargetPointAuto(depositX-6, depositY-2, depositTheta);
//                }
                robot.drivetrain.setTargetPointAuto(depositX, depositY, depositTheta);

                // if robot is at foundation, deposit stone, end segment
                if (robot.drivetrain.isAtPoseAuto(depositX,depositY,depositTheta) || time.seconds() > 2 /*&& Math.abs((robot.drivetrain.y - robot.drivetrain.lasty)/(time.seconds()-lastime))<0.1*/) {
//                    if (robot.stoneInRobot){
//                        robot.depositAuto();
//                    }
                    stoneCounter++; // increase stone counter

                    // if robot has collected 3rd or 4th stone, get another stoone
                    if (stoneCounter == 1 || stoneCounter == 2) {
                        toQuarryStone3_5 = false;
                        skystoneStone3_5 = false;
                        backToCenterStone3_5 = false;
                        toFoundationStone3_5 = false;
                        log("toquarry");
                    }
                    // if robot has collected 5th stone, score foundation
                    else if (stoneCounter == 3){
                        toFoundationStone3_5 = true;
                        log("scorefound");
                    }
                    time.reset();
                }
                //lastime = time.seconds();
            }

            // score foundation
            else if (!scoreFoundation) {
                robot.drivetrain.setTargetPointAuto(30, 25, Math.PI/2);

                // if robot is at foundation, deposit stone, end segment
                if (robot.drivetrain.isAtPoseAuto(30,25,Math.PI/2) || time.seconds() > 1.5) {
                    scoreFoundation = true;
                    time.reset(); log("totape");
                }
            }

            // park at tape under the alliance skybridge
            else if (!toTape) {
                //if(robot.stacker.isArmHome() && !robot.stacker.stoneClamped){
                    robot.drivetrain.setTargetPointAuto(30, 72, Math.PI / 2);
                //}
//                else if(time.seconds()>3){
//                    robot.letGo = true;
//                }
            }

            else {
                robot.drivetrain.setControls(0,0,0);
            }

            telemetry.addData("skystone position", skystonePos);
            telemetry.addData("x", robot.drivetrain.x);
            telemetry.addData("y", robot.drivetrain.y);
            telemetry.addData("theta", robot.drivetrain.currentheading);
            telemetry.update();
        }

        robot.update();
        robot.logger.flush();
        robot.logger.stopLogging();
        detector.interrupt();
    }

    public void log(String message) {
        Log.w("auto", " ");
        Log.w("auto", message + " -------------------------------------------------------------------------");
    }
}