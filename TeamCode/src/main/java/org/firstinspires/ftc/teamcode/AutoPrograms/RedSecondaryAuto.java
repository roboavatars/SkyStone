//package org.firstinspires.ftc.teamcode.AutoPrograms;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.RobotClasses.Robot;
//@Autonomous
//public class RedSecondaryAuto extends LinearOpMode {
//
//    private Robot robot;
//
//    @Override
//    public void runOpMode() {
//
//        // initialize robot
//        robot = new Robot(this, 9, 73, Math.PI/2, true);
//        robot.logger.startLogging();
//        robot.grabber.releaseFoundation();
//
//        robot.stacker.unClampStone();
//        robot.stacker.goHome();
//
//        // after start
//        waitForStart();
//
//        ElapsedTime time = new ElapsedTime();
//
//        // robot move loop
//        while (opModeIsActive()) {
//            robot.update();
//
//            robot.drivetrain.setTargetPoint(9,65,Math.PI/2);
//            robot.intakeManual = true;
//            robot.intake.setControls(0);
//        }
//
//        robot.update();
//        robot.logger.stopLogging();
//    }
//}