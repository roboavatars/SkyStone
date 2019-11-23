package org.firstinspires.ftc.teamcode.AutoPrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.skyStoneDetector;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.Splines.Spline;
import org.firstinspires.ftc.teamcode.Splines.SplineGenerator;

@Autonomous
public class BlueAuto extends LinearOpMode {

    private Robot robot;
    private skyStoneDetector detector = new skyStoneDetector(this);

    @Override
    public void runOpMode() {

        // initialize skystone detector
        detector.initializeCamera();
        detector.start();
        detector.setActive(true);
        detector.isAllianceRed(false);

        // initialize robot
        robot = new Robot(this, 135, 111, 0);
        robot.logger.startLogging();
        robot.grabber.releaseFoundation();
        robot.intake.setControls(0);
        robot.stacker.unClampStone();
        robot.stacker.goHome();

        // after start
        waitForStart();

        // skystone position variables
        double skystonePos = detector.getPosition();
        double skystoneY = robot.drivetrain.y;

        // segment finished variables
        boolean skystone1 = false;
        boolean backToCenter1 = false;
        boolean toFoundation1 = false;
        boolean foundationTurn = false;
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
        double foundationTurnTime = 1.75;
        double toQuarryTime = 3;
        double skystone2Time = 2;

        // set skystone y coordinate according to skystone position
        if (skystonePos == 1) {
            skystoneY = 131;
        } else if (skystonePos == 2) {
            skystoneY = 122;
        } else if (skystonePos == 3) {
            skystoneY = 113;
        }

        // generate splines
        SplineGenerator splineGenerator = new SplineGenerator();
        Spline[] skystone1Spline = splineGenerator.SplineBetweenTwoPoints(135, 111,
                99, skystoneY, 0, 3 * Math.PI / 4, 0, 0,
                20, 0, 0, 0, skystone1Time);

        Spline[] backToCenterSpline = splineGenerator.SplineBetweenTwoPoints(99, skystoneY,
                111, skystoneY - 12, 3 * Math.PI / 4, Math.PI / 2, 0, -70,
                -20, -50, 0, 0, backToCenterTime);
        Spline backToCenterThetaSpline = new Spline(3 * Math.PI / 4, 0, 0, Math.PI / 2, 0, 0, backToCenterTime);

        Spline[] foundationTurnSpline = splineGenerator.SplineBetweenTwoPoints(108, 55,
                113, 36, Math.PI / 2, 0, -70, 0,
                -50, 0, 0, 0, foundationTurnTime);
        Spline foundationTurnThetaSpline = new Spline(Math.PI / 2, 0, 0, 0, 0, 0, foundationTurnTime);

        Spline[] toQuarrySpline = splineGenerator.SplineBetweenTwoPoints(109, 29,
                120, skystoneY - 30, Math.PI / 2, 3 * Math.PI / 4, 0, 0,
                20, 0, 0, 0, toQuarryTime);
        Spline toQuarryThetaSpline = new Spline(Math.PI / 2, 0, 0, 3 * Math.PI / 4, 0, 0, toQuarryTime);

        Spline[] skystone2Spline = splineGenerator.SplineBetweenTwoPoints(120, skystoneY - 30,
                99, skystoneY - 26, Math.PI / 2, 3 * Math.PI / 4, 30, 0,
                20, 0, 0, 0, skystone2Time);

        // time used to end segments after a certain period of time
        ElapsedTime time = new ElapsedTime();

        robot.intake.setControls(-1);

        // robot move loop
        while (opModeIsActive()) {

            // update robot's coordinates on field from odometry pods
            robot.update();

            // get the first skystone
            if (!skystone1) {
                double currentTime = Math.min(2, time.seconds());

                if (time.seconds() < skystone1Time) {
                    robot.drivetrain.setTargetPoint(skystone1Spline[0].position(currentTime), skystone1Spline[1].position(currentTime),
                            3 * Math.PI / 4 + 0.1);
                }
                // if skystone is clamped or robot has been trying to intake stone for too long, move on
                else if (robot.stacker.stoneClamped || time.seconds() > skystone1Time + 2) {
                    skystone1 = true;
                    detector.setActive(false);
                    backToCenterSpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            111, skystoneY - 12, robot.drivetrain.currentheading, Math.PI / 2, 0, -70,
                            -20, -50, 0, 0, backToCenterTime);
                    backToCenterThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI / 2, 0, 0, backToCenterTime);
                    time.reset();
                }
                // if skystone has not been clamped, move forward to try to suck it in
                else {
                    robot.drivetrain.setTargetPoint(robot.drivetrain.x - 1, robot.drivetrain.y + 1, robot.drivetrain.currentheading);
                }
            }

            // go to the center of the tile closet to the neutral skybridge to avoid hitting alliance partner's robot
            else if (!backToCenter1) {
                double currentTime = Math.min(backToCenterTime, time.seconds());
                robot.drivetrain.setTargetPoint(backToCenterSpline[0].position(currentTime), backToCenterSpline[1].position(currentTime),
                        backToCenterThetaSpline.position(currentTime));

                if (time.seconds() > backToCenterTime) {
                    backToCenter1 = true;
                    time.reset();
                }
            }

            // get near the foundation
            else if (!toFoundation1) {
                robot.drivetrain.setTargetPoint(108, 55, Math.PI / 2);
                if (robot.drivetrain.isAtPose(108, 55, Math.PI / 2)) {
                    toFoundation1 = true;
                    foundationTurnSpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            113, 36, robot.drivetrain.currentheading, 0, -70, -30,
                            -50, -20, 0, 0, foundationTurnTime);
                    foundationTurnThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, 0, 0, 0, foundationTurnTime);

                    time.reset();
                }
            }

            // turn robot to align with foundation
            else if (!foundationTurn) {
                double currentTime = Math.min(foundationTurnTime, time.seconds());
                robot.drivetrain.setTargetPoint(foundationTurnSpline[0].position(currentTime), foundationTurnSpline[1].position(currentTime),
                        foundationTurnThetaSpline.position(currentTime));
                if (time.seconds() > foundationTurnTime) {
                    foundationTurn = true;
                    time.reset();
                }
            }

            // approach and align robot with foundation
            else if (!approachFoundation) {
                robot.drivetrain.setTargetPoint(100, 25, 0);

                // grab foundation
                if (time.seconds() > 0.5) {
                    robot.grabber.grabFoundation();
                }
                // extend arm with skystone over the foundation
                if (robot.drivetrain.isAtPose(100, 25, 0) && time.seconds() > 1.5) {
                    approachFoundation = true;
                    if (robot.stacker.stoneClamped) {
                        robot.stacker.setLevel(1);
                        robot.swapArmState();
                        robot.deposit();
                    }
                    time.reset();
                }

            }

            // pull the foundation so that it is in front of the building site
            else if (!pullFoundation) {
                robot.drivetrain.setTargetPoint(118, 25, 0, 0.8, 0, 0.8);
                if (robot.drivetrain.isAtPose(118, 25, 0)) {
                    pullFoundation = true;
                    time.reset();
                }
            }

            // turn the foundation 90 degrees
            else if (!turnFoundation) {
                robot.drivetrain.setTargetPoint(109, 35, Math.PI / 2, 0, 0, 3);
                if (robot.drivetrain.isAtPose(109, 35, Math.PI / 2, 144, 144, 0.1)) {
                    turnFoundation = true;
                    time.reset();
                }
            }

            // push the foundation forward to score it in building zone
            else if (!pushFoundation) {
                robot.drivetrain.setTargetPoint(109, 29, Math.PI / 2, 0.1, 0.4, 0.8);
                // release the skystone onto the foundation
                robot.stacker.unClampStone();

                if (time.seconds() > 1) {
                    pushFoundation = true;// retract the arm back into the robot, release foundation
                    // retract the arm back into the robot, release foundation
                    if (robot.stacker.isArmOut()) {
                        robot.swapArmState();
                    }
                    robot.grabber.releaseFoundation();
                    toQuarrySpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            120, skystoneY - 30, robot.drivetrain.currentheading, 3 * Math.PI / 4, 0, 0,
                            20, 0, 0, 0, toQuarryTime);
                    toQuarryThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, 3 * Math.PI / 4, 0, 0, toQuarryTime);
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
                    skystone2Spline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            99, skystoneY - 26, robot.drivetrain.currentheading, 3 * Math.PI / 4, 30, 0,
                            20, 0, 0, 0, skystone2Time);
                    time.reset();
                }
            }

            // get the second skystone
            else if (!skystone2) {
                double currentTime = Math.min(skystone2Time, time.seconds());

                if (time.seconds() < skystone2Time) {
                    robot.drivetrain.setTargetPoint(skystone2Spline[0].position(currentTime), skystone2Spline[1].position(currentTime),
                            3 * Math.PI / 4 + 0.15);
                }
                // if skystone is clamped or robot has been trying to intake stone for too long, move on
                else if (robot.stacker.stoneClamped || time.seconds() > skystone2Time + 2) {
                    skystone2 = true;
                    time.reset();
                }
                // if skystone has not been clamped, move forward to try to suck it in
                else {
                    robot.drivetrain.setTargetPoint(robot.drivetrain.x - 1, robot.drivetrain.y + 1, robot.drivetrain.currentheading);
                }
            }

            // go to the center of the tile closet to the neutral skybridge to avoid hitting alliance partner's robot
            else if (!backToCenter2) {
                robot.drivetrain.setTargetPoint(111, 85, Math.PI / 2, 0.2, 0.2, 0.8);
                if (robot.drivetrain.y < 85) {
                    backToCenter2 = true;
                    time.reset();
                }
            }

            // go to foundation to deposit second skystone
            else if (!toFoundation2) {
                robot.drivetrain.setTargetPoint(111, 33, Math.PI / 2, 0.2, 0.03, 0.8);
                // extend arm with skystone over the foundation
                if (robot.drivetrain.isAtPose(robot.drivetrain.x, 60, robot.drivetrain.currentheading)) {
                    robot.stacker.setLevel(1);
                    robot.swapArmState();
                }
                // release the skystone onto the foundation
                if (time.seconds() > 2.5) {
                    robot.stacker.unClampStone();
                }
                if (robot.drivetrain.isAtPose(111, 33, Math.PI / 2) && time.seconds() < 3) {
                    robot.stacker.unClampStone();
                    toFoundation2 = true;
                    // retract the arm back into the robot
                    if (robot.stacker.isArmOut()) {
                        robot.swapArmState();
                    }
                    time.reset();
                }
            }

            // park at tape under the alliance skybridge
            else if (!toTape) {
                robot.drivetrain.setTargetPoint(114, 62, Math.PI / 2, 0.2, 0.2, 0.8);
                if (robot.drivetrain.isAtPose(144, 62, Math.PI / 2)) {
                    toTape = true;
                    time.reset();
                }
            }

            // stop robot
            else {
                robot.drivetrain.setControls(0, 0, 0);
                detector.interrupt();
            }

            telemetry.addData("skystone position", skystonePos);
            telemetry.addData("x", robot.drivetrain.x);
            telemetry.addData("y", robot.drivetrain.y);
            telemetry.addData("theta", robot.drivetrain.currentheading);
            telemetry.update();
        }

        robot.update();
        robot.logger.writePos(robot.drivetrain.x, robot.drivetrain.y, robot.drivetrain.currentheading);
        robot.logger.flush();
        robot.logger.stopLogging();
    }
}