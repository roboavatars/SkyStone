package org.firstinspires.ftc.teamcode.AutoPrograms;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.skyStoneDetector;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.Splines.Spline;
import org.firstinspires.ftc.teamcode.Splines.SplineGenerator;

@Autonomous (name = "2 block+ red auto")
public class RedAuto extends LinearOpMode {

    private Robot robot;
    private skyStoneDetector detector = new skyStoneDetector(this);

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();

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
        boolean toQuarry2 = false;
        boolean skystone2 = false;
        boolean backToCenter2 = false;
        boolean toFoundation2 = false;
        boolean toQuarry3_5 = false;
        boolean stone3_5 = false;
        boolean backToCenter3_5 = false;
        boolean toFoundation3_5 = false;
        boolean scoreFoundation = false;
        boolean toTape = false;

        // spline time variables
        double skystoneTime = 2;
        double backToCenterTime = 0.75;
        double toFoundationTime = 2;
        double foundationPullTime = 1.75;
        double toQuarryTime = 1.5;
        double toFoundation2Time = 1.5;
        double stone3_5Time = 2;
        double toFoundation3_5Time = 1.5;

        int stoneCounter = 0;

        double[] sspos1 = {52,95,Math.PI/3, 54,119,Math.PI/2, 56,131,Math.PI/2, 0,0,0};
        double[] sspos2 = {52,95,Math.PI/3, 54,119,Math.PI/2, 56,131,Math.PI/2, 0,0,0};
        double[] sspos3 = {52,95,Math.PI/3, 54,119,Math.PI/2, 56,131,Math.PI/2, 0,0,0};

        double[] stonearray = {};

        int ss2Y = 34;

        // set skystone y coordinate according to skystone position
        if (skystonePos == 1) {
            skystoneY = 129; ss2Y = 35; stonearray = sspos1;
            skystoneTime += 0;
            toQuarryTime += 0;
            backToCenterTime += 0;
        }
        else if (skystonePos == 2) {
            skystoneY = 121; stonearray = sspos2;
        }
        else if (skystonePos == 3) {
            skystoneY = 112; stonearray = sspos3;
            skystoneTime -= 0;
            toQuarryTime -= 0;
            backToCenterTime -= 0;
        }

        double stoneX = stonearray[0], stoneY = stonearray[1], stoneTheta = stonearray[2];
        detector.setActive(false);

        // generate splines
        SplineGenerator splineGenerator = new SplineGenerator();
        Spline[] skystone1Spline = splineGenerator.SplineBetweenTwoPoints(9, 111,
                45, skystoneY, 0, Math.PI / 4 + 0.2, 0, 0,
                100, -100, 0, 0, skystoneTime);
        Spline skystone1ThetaSpline = new Spline(0, 0, 8, Math.PI / 4 + 0.2, 0, 0, skystoneTime);

        Spline[] backToCenterSpline = splineGenerator.SplineBetweenTwoPoints(45, skystoneY,
                30, skystoneY - 12, Math.PI / 4, Math.PI / 2, 0, -70,
                -50, -100, 0, 0, backToCenterTime);
        Spline backToCenterThetaSpline = new Spline(Math.PI / 4, 0, 0, Math.PI / 2, 0, 0, backToCenterTime);

        Spline[] toFoundationSpline = splineGenerator.SplineBetweenTwoPoints(30, skystoneY-12,
                36, 55, Math.PI / 2, Math.PI / 2, 0, -70,
                -50, -100, 0, 0, toFoundationTime);
        Spline toFoundationThetaSpline = new Spline(Math.PI / 2, 0, 0, Math.PI / 2, 0, 0, toFoundationTime);

        Spline[] toQuarrySpline = splineGenerator.SplineBetweenTwoPoints(35 , 55,
                24, skystoneY - 30, Math.PI / 2, Math.PI / 4, -70, 0,
                -20, 0, 0, 0, toQuarryTime);
        Spline toQuarryThetaSpline = new Spline(Math.PI / 2, 0, 0, Math.PI / 4, 0, 0, toQuarryTime);

        Spline[] skystone2Spline = splineGenerator.SplineBetweenTwoPoints(24, skystoneY - 30,
                45, skystoneY - 26, Math.PI / 2, Math.PI / 4, 50, 50,
                75, 75, 0, 0, skystoneTime);
        Spline skystone2ThetaSpline = new Spline(0, 0, 8, Math.PI / 4 + 0.2, 0, 0, skystoneTime);

        Spline[] stone3_5Spline = splineGenerator.SplineBetweenTwoPoints(24, skystoneY - 30,
                stoneX, stoneY, Math.PI / 2, stoneTheta, 50, 50,
                75, 75, 0, 0, skystoneTime);
        Spline stone3_5ThetaSpline = new Spline(0, 0, 8, Math.PI / 4 + 0.2, 0, 0, skystoneTime);

        // time used to end segments after a certain period of time
        ElapsedTime time = new ElapsedTime();

        //double lastime = 0;
        robot.intake.setControls(0.6);
        sleep(33);

        log("ss1");

        // robot move loop
        while (opModeIsActive()) {

            TelemetryPacket packet = new TelemetryPacket();

            // update robot's location and states
            robot.update();

            // get the first skystone
            if (!skystone1) {
                double currentTime = Math.min(skystoneTime, time.seconds());

                // if less than moving time, continue moving
                if (time.seconds() < skystoneTime) {
                    robot.drivetrain.setTargetPointAuto(skystone1Spline[0].position(currentTime), skystone1Spline[1].position(currentTime),
                            skystone1ThetaSpline.position(currentTime));
                }
                // if skystone is clamped or robot has been trying to intake stone for too long, move on
                else if (robot.stoneInRobot || time.seconds() > skystoneTime + 1) {
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
                if (robot.drivetrain.y < 58 || time.seconds() > toFoundationTime) {
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
                    //if (robot.stacker.stoneClamped) {
                      //  robot.depositAuto();
                    //}
                    robot.grabber.grabFoundation();

                    time.reset(); log("appfound");
                }
            }

            // approach and align robot with foundation
            else if (!approachFoundation) {
                robot.drivetrain.setTargetPointAuto(47, 18, Math.PI);

                // if at position or time met, end segment
                if (robot.drivetrain.isAtPoseAuto(47, 18, Math.PI) && time.seconds() > 0.5) {
                    approachFoundation = true;
                    time.reset(); log("bringfound");
                }
            }

            // turn and pull foundation
            else if (!foundationPull) {
                if (robot.drivetrain.currentheading > 2*Math.PI/3) {
                    robot.drivetrain.setControls(0,0.6,-0.35);
                } else {
                    robot.drivetrain.setTargetPoint(30,45,Math.PI/2);
                }

            // if arm is home or time met, release foundation, end segment
                if (time.seconds() > foundationPullTime || (robot.drivetrain.y > 40 && robot.drivetrain.x > 28)) {
                    depositX = robot.drivetrain.x; depositY = robot.drivetrain.y; depositTheta = robot.drivetrain.currentheading;
                    Log.w("auto", "Deposit Cor: " + depositX + " " + depositY + " " + depositTheta);
                    foundationPull = true;

                    robot.grabber.releaseFoundation();
                    toQuarrySpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            31, skystoneY-ss2Y, robot.drivetrain.currentheading, robot.drivetrain.currentheading, 20, 60,
                            100, 0, 0, 0, skystoneTime);
                    toQuarryThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI/2, 0, 0, skystoneTime);
                    time.reset(); log("toquarry2");
                }
            }

            // travel back to the quarry to get second skystone
            else if (!toQuarry2) {
                double currentTime = Math.min(toQuarryTime, time.seconds());
                robot.drivetrain.setTargetPointAuto(toQuarrySpline[0].position(currentTime), toQuarrySpline[1].position(currentTime),
                        toQuarryThetaSpline.position(currentTime), 0.2, 0.2, 1.2);

                // if time is met, end segment
                if (time.seconds() > toQuarryTime) {
                    toQuarry2 = true;
                    skystone2Spline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            49, skystoneY - 19, robot.drivetrain.currentheading, robot.drivetrain.currentheading, 60, 30,
                            0, -30, 0, 0, skystoneTime);
                    skystone2ThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI / 4 + 0.2, 0, 0, skystoneTime);
                    time.reset(); log("ss2");
                }
            }

            // get the second skystone
            else if (!skystone2) {
                double currentTime = Math.min(skystoneTime, time.seconds());
                robot.drivetrain.setTargetPointAuto(skystone2Spline[0].position(currentTime), skystone2Spline[1].position(currentTime),
                        skystone2ThetaSpline.position(currentTime));

                // if skystone is in robot or time is met, end segment
                if (robot.stoneInRobot || time.seconds() > skystoneTime + 1) {
                    skystone2 = true;
                    backToCenterSpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            30, skystoneY - 32, robot.drivetrain.currentheading, robot.drivetrain.currentheading, 0, -60,
                            20, 0, 0, 0, backToCenterTime);
                    backToCenterThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI / 2, 0, 0, backToCenterTime);
                    time.reset(); log("backcenter2");
                }
            }

            else if (!backToCenter2) {
                double currentTime = Math.min(backToCenterTime,time.seconds());
                robot.drivetrain.setTargetPointAuto(backToCenterSpline[0].position(currentTime), backToCenterSpline[1].position(currentTime),
                        backToCenterThetaSpline.position(currentTime));

                //robot.grabber.extendRangeSensor();
                if (time.seconds() > backToCenterTime) {
                    backToCenter2 = true;
                    toFoundationSpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            depositX, depositY, robot.drivetrain.currentheading, robot.drivetrain.currentheading, 40, 60,
                            100, -60, 0, 0, toFoundation2Time);
                    toFoundationThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI/2, 0, 0, toFoundation2Time);

                    time.reset(); log("tofound2");
                }
            }

            // go to foundation to deposit second skystone
            else if (!toFoundation2) {
                //if(!robot.isAutoAlign){
                //  robot.drivetrain.setTargetPointAuto(depositX-6, depositY-2, depositTheta);
                //}

                double currentTime = Math.min(toFoundation2Time,time.seconds());
                robot.drivetrain.setTargetPointAuto(toFoundationSpline[0].position(currentTime), toFoundationSpline[1].position(currentTime),
                        toFoundationThetaSpline.position(currentTime));
                if (robot.drivetrain.isAtPoseAuto(depositX,depositY,Math.PI/2) || time.seconds() > toFoundation2Time + 1) {
                    //if (robot.stoneInRobot){
                    //    robot.depositAuto();
                    //}
                    toFoundation2 = true;
                    toQuarrySpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            31, skystoneY-34, robot.drivetrain.currentheading, robot.drivetrain.currentheading, 20, 60,
                            100, 0, 0, 0, toQuarryTime);
                    toQuarryThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI / 2, 0, 0, toQuarryTime);
                    time.reset(); log("toquarry3-5");
                }
            }
            
            else if (!toQuarry3_5) {
                double currentTime = Math.min(toQuarryTime, time.seconds());
                robot.drivetrain.setTargetPointAuto(toQuarrySpline[0].position(currentTime), toQuarrySpline[1].position(currentTime),
                        toQuarryThetaSpline.position(currentTime), 0.2, 0.2, 1.2);
                
                if (time.seconds() > toQuarryTime) {
                    toQuarry3_5 = true;
                    stone3_5Spline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            stoneX, stoneY, robot.drivetrain.currentheading, robot.drivetrain.currentheading, 60, 30,
                            0, -30, 0, 0, stone3_5Time);
                    stone3_5ThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, stoneTheta, 0, 0, stone3_5Time);
                    time.reset(); log("s3-5");
                }
            }

            else if (!stone3_5) {
                double currentTime = Math.min(stone3_5Time, time.seconds());
                robot.drivetrain.setTargetPointAuto(stone3_5Spline[0].position(currentTime), stone3_5Spline[1].position(currentTime),
                        stone3_5ThetaSpline.position(currentTime));

                // if skystone is in robot or time is met, end segment
                if (robot.stoneInRobot || time.seconds() > stone3_5Time + 1) {
                    stone3_5 = true;
                    if (stoneCounter <= 6) {
                        stoneCounter += 3;
                        stoneX = stonearray[stoneCounter]; stoneY = stonearray[stoneCounter+1]; stoneTheta = stonearray[stoneCounter+2];
                    }
                    backToCenterSpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            29, skystoneY-32, robot.drivetrain.currentheading, robot.drivetrain.currentheading, 0, -60,
                            20, 0, 0, 0, backToCenterTime);
                    backToCenterThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI / 2, 0, 0, backToCenterTime);
                    time.reset(); log("backcenter3-5");
                }
            }

            else if (!backToCenter3_5) {
                double currentTime = Math.min(backToCenterTime,time.seconds());
                robot.drivetrain.setTargetPointAuto(backToCenterSpline[0].position(currentTime), backToCenterSpline[1].position(currentTime),
                        backToCenterThetaSpline.position(currentTime));

                //robot.grabber.extendRangeSensor();
                if (time.seconds() > backToCenterTime) {
                    backToCenter3_5 = true;
                    if (stoneCounter <= 9) {
                        toFoundation3_5 = false;
                    }
                    toFoundationSpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            depositX, depositY, robot.drivetrain.currentheading, robot.drivetrain.currentheading, 40, 60,
                            100, -60, 0, 0, toFoundation3_5Time);
                    toFoundationThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI/2, 0, 0, toFoundation3_5Time);

                    time.reset(); log("tofound3-5");
                }
            }

            else if (!toFoundation3_5) {
                //if(!robot.isAutoAlign){
                //  robot.drivetrain.setTargetPointAuto(depositX-6, depositY-2, depositTheta);
                //}

                double currentTime = Math.min(toFoundation3_5Time,time.seconds());
                robot.drivetrain.setTargetPointAuto(toFoundationSpline[0].position(currentTime), toFoundationSpline[1].position(currentTime),
                        toFoundationThetaSpline.position(currentTime));
                if (robot.drivetrain.isAtPoseAuto(depositX,depositY,Math.PI/2) || time.seconds() > toFoundation2Time + 1) {
                    //if (robot.stoneInRobot){
                    //    robot.depositAuto();
                    //}
                    toFoundation3_5 = true;
                    if (stoneCounter <= 6) {
                        toQuarry3_5 = false;
                        stone3_5 = false;
                        backToCenter3_5 = false;
                    }
                    toQuarrySpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            31, skystoneY-ss2Y, robot.drivetrain.currentheading, robot.drivetrain.currentheading, 20, 60,
                            100, 0, 0, 0, toQuarryTime);
                    toQuarryThetaSpline = new Spline(robot.drivetrain.currentheading, 0, 0, Math.PI / 2, 0, 0, toQuarryTime);
                    time.reset(); log("toquarry3-5");
                }
            }

//            // score foundation
//            else if (!scoreFoundation) {
//                robot.drivetrain.setTargetPointAuto(30, 25, Math.PI/2);
//
//                // if robot is at foundation, deposit stone, end segment
//                if (robot.drivetrain.isAtPoseAuto(30,25,Math.PI/2) || time.seconds() > 1.5) {
//                    scoreFoundation = true;
//                    time.reset(); log("totape");
//                }
//            }

            // park at tape under the alliance skybridge
            else if (!toTape) {
//                //if(robot.stacker.isArmHome() && !robot.stacker.stoneClamped){
                    robot.drivetrain.setTargetPointAuto(30, 72, Math.PI / 2);
//                //}
//                //else if(time.seconds()>3){
//                  //  robot.letGo = true;
//                //}
            }

            else {
                robot.drivetrain.setControls(0,0,0);
            }

            //telemetry.addData("skystone position", skystonePos);
            //telemetry.addData("x", robot.drivetrain.x);
            //telemetry.addData("y", robot.drivetrain.y);
            //telemetry.addData("theta", robot.drivetrain.currentheading);
            //telemetry.update();

            packet.put("skystone position", skystonePos);
            packet.put("x", robot.drivetrain.x);
            packet.put("y", robot.drivetrain.y);
            packet.put("theta", robot.drivetrain.currentheading);
            packet.put("battery voltage", getBatteryVoltage());
            packet.put("time", (System.currentTimeMillis()-robot.startTime)/1000);
            dashboard.sendTelemetryPacket(packet);
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