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
        robot = new Robot(hardwareMap,this, 9,111,0);

        //after start
        waitForStart();

        double skystonePos = detector.getPosition();
        double skystoneY = robot.drivetrain.y;
        boolean skystone1 = false;
        boolean backToCenter = false;
        boolean toFoundation = false;
        
        
        if (skystonePos == 1) {
            skystoneY = 129;

        } else if (skystonePos == 2) {
            skystoneY = 121;

        } else if (skystonePos == 3) {
            skystoneY = 113;
        }
        
        SplineGenerator splineGenerator = new SplineGenerator();
        Spline[] skystone1Splines = splineGenerator.SplineBetweenTwoPoints(9,111,
                45, skystoneY,0,Math.PI/4,0,0,
                20,0,0,0, 2);
        
        Spline[] backToCenterSpline = splineGenerator.SplineBetweenTwoPoints(45, skystoneY,
                36, skystoneY - 12, Math.PI/4, Math.PI / 2, 0, 40,
                20, 20, 0, 0, 1.5);
 
        ElapsedTime time = new ElapsedTime();

        while (opModeIsActive()) {
            robot.update();

            
            if (!skystone1){
                //turn intake on
                robot.intake.setControls(1);
    
                double currentTime = Math.min(2, time.seconds());
    
                robot.drivetrain.setTargetPoint(skystone1Splines[0].position(currentTime), skystone1Splines[1].position(currentTime),
                        Math.PI/4);
                
                if (time.seconds() > 4) {
                    skystone1 = true;
                    robot.intake.setControls(0);
                    backToCenterSpline = splineGenerator.SplineBetweenTwoPoints(robot.drivetrain.x, robot.drivetrain.y,
                            36, skystoneY - 12, robot.drivetrain.currentheading, Math.PI / 2, 0, 0,
                            20, 0, 0, 0, 1.5);
                    time.reset();
    
                }
            } else if (!backToCenter){
                double currentTime = Math.min(1.5, time.seconds());
    
                robot.drivetrain.setTargetPoint(backToCenterSpline[0].position(currentTime), backToCenterSpline[1].position(currentTime),
                        Math.atan(backToCenterSpline[1].velocity(currentTime)/backToCenterSpline[0].velocity(currentTime)));
    
    
    
            } else if (!toFoundation){
            
            }
            


            telemetry.addData("skystone position", skystonePos);
            telemetry.addData("x", robot.drivetrain.x);
            telemetry.addData("y", robot.drivetrain.y);
            telemetry.addData("theta", robot.drivetrain.currentheading);
            telemetry.update();
        }
        detector.setActive(false);
    }
}