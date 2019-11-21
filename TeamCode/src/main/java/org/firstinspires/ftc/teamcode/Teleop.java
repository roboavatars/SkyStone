package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotClasses.Robot;

@TeleOp(name="Teleop") @SuppressWarnings("FieldCanBeLocal")
public class Teleop extends LinearOpMode {

    private Robot robot;

    private double intakePower = 0;
    private double armPower = 0;
    private double liftPower = 0;

    private final boolean robotCentric = false;
    private int z = 0;
    private int stackcounter = 0;
    private boolean armout = false;

    boolean dpadup = true;
    boolean dpaddown = true;

    @Override
    public void runOpMode() {
        robot = new Robot(this, 0 , 0,0);
        //robot.stacker.unClampStone();
        ElapsedTime xBuffer = new ElapsedTime();

        waitForStart();
        robot.drivetrain.resetAngle();
        robot.intake.setControls(0);
        robot.stacker.unClampStone();
        robot.stacker.setDepositControls(0.5,130);
        while (opModeIsActive()){

//            if (gamepad1.x && xBuffer.milliseconds()>300 && intakePower == 0) {
//                intakePower = 1;
//                xBuffer.reset();
//                xBuffer.startTime();
//            } else if (gamepad1.x && xBuffer.milliseconds()>300) {
//                intakePower = 0;
//                xBuffer.reset();
//                xBuffer.startTime();
//            }



            z++;
            if(z%100 == 0 && stackcounter == 0){
                double distance = robot.stoneSensor.getDistance(DistanceUnit.INCH);
                if (distance < 5) {
                    robot.stacker.setDepositControls(0.5,50);
                    robot.stacker.clampStone();
                    robot.intake.setControls(0);
                }
                else{
                    robot.intake.setControls(1);
                }
            }
            if(gamepad1.left_bumper){
                robot.stacker.unClampStone();
            }

            if(gamepad1.right_bumper && stackcounter == 0 && !armout){
                robot.stacker.deposit();
                armout = true;
                stackcounter = 100;
            }else if(gamepad1.right_bumper && stackcounter == 0 && armout){
                robot.stacker.setLiftControls(0.5,0);
                robot.stacker.setDepositControls(0.5,130);
                robot.stacker.nextLevel();
                stackcounter = 100;
                armout = false;
            }
            else if(stackcounter>0){
                stackcounter--;
            }


            if(gamepad1.dpad_up && dpadup){
                dpadup = false;

            }else if(!dpadup && !gamepad1.dpad_up){
                robot.stacker.nextLevel();
                dpadup = true;
            }

            if(gamepad1.dpad_down && dpaddown){
                dpaddown = false;

            }else if(!dpaddown && !gamepad1.dpad_down){
                robot.stacker.lastLevel();
                dpaddown = true;
            }



            
            if (gamepad1.dpad_left) robot.grabber.grabFoundation();
            if (gamepad1.dpad_right) robot.grabber.releaseFoundation();

    
            if (robotCentric) {
                robot.drivetrain.setControls(-gamepad1.left_stick_x, -gamepad1.left_stick_y, -gamepad1.right_stick_x);
            }
            else {robot.drivetrain.setGlobalControls(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);}
            robot.update();


//            telemetry.addData("X", robot.drivetrain.x);
//            telemetry.addData("Y", robot.drivetrain.y);
//            telemetry.addData("Theta", robot.drivetrain.currentheading);
            telemetry.update();
        }
    }
}