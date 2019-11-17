package org.firstinspires.ftc.teamcode.AutoPrograms;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.OpenCV.skyStoneDetector;
import org.firstinspires.ftc.teamcode.RobotClasses.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
import org.firstinspires.ftc.teamcode.Splines.Spline;
import org.firstinspires.ftc.teamcode.Splines.SplineGenerator;

@Autonomous
public class RedAuto extends LinearOpMode {

    private double ssAligned = 125;
    private double curPos;
    private Robot robot;

    skyStoneDetector detector = new skyStoneDetector(this);

    @Override
    public void runOpMode() throws InterruptedException {
        //initializing skystone detector stuff after init
        detector.initializeCamera();
        detector.start();
        detector.setActive(true);
        detector.isAllianceRed(true);

        //initializing mecanum dt
        robot = new Robot(hardwareMap,this, 9,111,0);

        //after start
        waitForStart();

        double skystonepos = detector.getPosition();

        double skystoney = robot.drivetrain.y;

        if(detector.getPosition() == 1){
            skystoney = 130;

        }else if(detector.getPosition() == 2){
            skystoney = 122;

        }else if(detector.getPosition() == 3){
            skystoney = 114;
        }
        SplineGenerator splineGenerator = new SplineGenerator();
        Spline[] splines = splineGenerator.SplineBetweenTwoPoints(9,111,47,
                skystoney,0,2*Math.PI/5,0,0,
                40,0,0,0, 2);
        Spline xspline = splines[0];
        Spline yspline = splines[1];

        ElapsedTime time = new ElapsedTime();

        while(opModeIsActive()){
            robot.drivetrain.updatePose();

            //turn intake on
            robot.intake.setControls(1);

            double currenttime = Math.min(2,time.seconds());
            robot.drivetrain.setTargetPoint(xspline.position(currenttime), yspline.position(currenttime),
                        Math.atan(yspline.velocity(currenttime)/xspline.velocity(currenttime)));


            telemetry.addData("skystone position",skystonepos);
            telemetry.addData("x", robot.drivetrain.x);
            telemetry.addData("y", robot.drivetrain.y);
            telemetry.addData("theta", robot.drivetrain.currentheading);
            telemetry.update();

        }


    }
}