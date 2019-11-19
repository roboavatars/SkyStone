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
        //initializing skystone detector stuff after init
        detector.initializeCamera();
        detector.start();
        detector.setActive(true);
        detector.isAllianceRed(true);

        //initializing robot
        robot = new Robot(this, 9,111,0);
        robot.grabber.releaseFoundation();

        //after start
        waitForStart();

        double skystonePos = detector.getPosition();
        double skystoneY = robot.drivetrain.y;
        
        boolean skystone1 = false;
        boolean backToCenter1 = false;
        boolean toFoundation1 = false;
        boolean approachFoundation = false;
        boolean pullFoundation = false;
        boolean turnFoundation = false;
        boolean pushFoundation = false;
        boolean toQuarry = false;
        boolean skystone2 = false;
        boolean toFoundation2 = false;
        boolean toTape = false;
        
        double skystone1Time = 2.5;
        double backToCenter1Time = 1;
        double toFoundation1Time = 2;
        double scoreFoundationTime = 2;
        double skystone2Time = 2;
        
        if (skystonePos == 1) {
            skystoneY = 132;
        } else if (skystonePos == 2) {
            skystoneY = 123;
        } else if (skystonePos == 3) {
            skystoneY = 113;
        }
        
        SplineGenerator splineGenerator = new SplineGenerator();
        Spline[] skystone1Splines = splineGenerator.SplineBetweenTwoPoints(9,111,
                45, skystoneY,0,Math.PI/4,0,0,
                20,0,0,0, skystone1Time);
        
        Spline[] backToCenterSpline = splineGenerator.SplineBetweenTwoPoints(45, skystoneY,
                36, skystoneY - 12, Math.PI/4, Math.PI/2, 0, -20,
                -20, -20, 0, 0, backToCenter1Time);
        Spline backToCenterThetaSpline = new Spline(Math.PI/4,0,0,Math.PI/2,0,0,backToCenter1Time);
    
        Spline[] toFoundationSpline = splineGenerator.SplineBetweenTwoPoints(36, skystoneY-12,
                36, 25, Math.PI/2, Math.PI, 0, 0,
                -20, 0, 0, 0, toFoundation1Time);
        Spline toFoundationThetaSpline = new Spline(Math.PI/2,0,0,Math.PI,0,0,toFoundation1Time);
    
        Spline[] scoreFoundationSpline = splineGenerator.SplineBetweenTwoPoints(39.5, 24,
                26, 40, Math.PI, Math.PI/2, 0, 0,
                10, 0, 0, 0, toFoundation1Time);
        Spline scoreFoundationThetaSpline = new Spline(Math.PI,0,0,Math.PI/2,0,0,scoreFoundationTime);
        
        Spline[] skystone2Spline = splineGenerator.SplineBetweenTwoPoints(36, 72,
                45, skystoneY-24, Math.PI/2, Math.PI/4, 30, 0,
                20, 0, 0, 0, skystone2Time);
        Spline skystone2ThetaSpline = new Spline(Math.PI/2,0,0,Math.PI/4,0,0,skystone2Time);
    
        ElapsedTime time = new ElapsedTime();
        while (opModeIsActive()) {
            robot.update();

            
            if (!skystone1){
                //turn intake on
                robot.intake.setControls(1);
    
                double currentTime = Math.min(2, time.seconds());
                robot.drivetrain.setTargetPoint(skystone1Splines[0].position(currentTime), skystone1Splines[1].position(currentTime),
                        Math.PI/4);
                
                if (time.seconds() > skystone1Time + 1) {
                    skystone1 = true;
                    robot.intake.setControls(0);
                    detector.setActive(false); detector.interrupt();
                    backToCenterSpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            36, skystoneY - 12, robot.drivetrain.currentheading, Math.PI/2, 0, -20,
                            -20, -20, 0, 0, backToCenter1Time);
    
                    backToCenterThetaSpline = new Spline(robot.drivetrain.currentheading,0,0,Math.PI/2,0,0,backToCenter1Time);
    
                    time.reset();
                }
                
            } else if (!backToCenter1){
                double currentTime = Math.min(backToCenter1Time, time.seconds());
                robot.drivetrain.setTargetPoint(backToCenterSpline[0].position(currentTime), backToCenterSpline[1].position(currentTime),
                        backToCenterThetaSpline.position(currentTime));
    
                if (time.seconds() > backToCenter1Time) {
                    backToCenter1 = true;
                    toFoundationSpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            33, 36, robot.drivetrain.currentheading, Math.PI, -20, 0,
                            -20, 0, 0, 0, toFoundation1Time);
                    toFoundationThetaSpline = new Spline(robot.drivetrain.currentheading,0,0,Math.PI,0,0,toFoundation1Time);
                    time.reset();
                }
    
            } else if (!toFoundation1){
                double currentTime = Math.min(toFoundation1Time, time.seconds());
                robot.drivetrain.setTargetPoint(toFoundationSpline[0].position(currentTime), toFoundationSpline[1].position(currentTime),
                        toFoundationThetaSpline.position(currentTime));
    
                if (time.seconds() > toFoundation1Time+1) {
                    toFoundation1 = true;
                    time.reset();
                }
                
            } else if (!approachFoundation) {
                robot.drivetrain.setTargetPoint(44, 25, Math.PI);
                if (time.seconds() > 0.5) {
                    robot.grabber.grabFoundation();
                }
                if (time.seconds() > 2) {
                    approachFoundation = true;
                    scoreFoundationSpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            24, 40, robot.drivetrain.currentheading, Math.PI/2, 0, 0,
                            20, 0, 0, 0, toFoundation1Time);
                    time.reset();
                }
                
            } else if (!pullFoundation) {
                robot.drivetrain.setTargetPoint(26, robot.drivetrain.y, Math.PI,1.5,0,0.8);
    
                if (time.seconds() > toFoundation1Time + 1) {
                    pullFoundation = true;
                    time.reset();
        
                }
                
            } else if (!turnFoundation) {
                
                robot.drivetrain.setTargetPoint(35, 35, Math.PI/2,0,0,3);
                if (time.seconds() > 2) {
                    turnFoundation = true;
                    time.reset();
                }

            } else if (!pushFoundation) {
    
                robot.drivetrain.setTargetPoint(35, 29, Math.PI/2,0.1,0.4,0.8);
                if (time.seconds() > 1) {
                    pushFoundation = true;
                    robot.grabber.releaseFoundation();
                    time.reset();
                }
    
            } else if (!toQuarry) {
    
                robot.drivetrain.setTargetPoint(24, skystoneY-35, Math.PI/4,0.2,0.2,0.8);
                if (time.seconds()>2) {
                    toQuarry = true;
                    skystone2Spline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            45, skystoneY-25, robot.drivetrain.currentheading, Math.PI/4, 30, 0,
                            20, 0, 0, 0, skystone2Time);
                    
                    time.reset();
                    robot.intake.setControls(1);
                }
            } else if(!skystone2){

               double currentTime = Math.min(skystone2Time, time.seconds());
               robot.drivetrain.setTargetPoint(skystone2Spline[0].position(currentTime), skystone2Spline[1].position(currentTime),
                       Math.PI/4);

               if (time.seconds()>skystone2Time+1){
                   robot.intake.setControls(0);
                   skystone2 = true;
                   time.reset();
               }

           } else if (!toFoundation2) {
    
                robot.drivetrain.setTargetPoint(35, 27, Math.PI/2, 0.2, 0.2, 0.8);
                if (time.seconds() > 2) {
                    toFoundation2 = true;
                    time.reset();
                }
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
    }
}