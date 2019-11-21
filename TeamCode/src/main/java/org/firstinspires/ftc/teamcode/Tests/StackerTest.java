package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Stacker;

@TeleOp(name="Stacker Test") @SuppressWarnings("FieldCanBeLocal")
public class StackerTest extends LinearOpMode {

    private Stacker stacker;
    private double armPower = 0;
    private double liftPower = 0;
    
    private int liftHome = 0;  private int armHome = 0;
    private int lift1 = 0;     private int arm1 = 0;
    private int lift2 = 0;     private int arm2 = 0;
    private int lift3 = 0;     private int arm3 = 0;
    private int lift4 = 0;     private int arm4 = 0;
    private int lift5 = 0;     private int arm5 = 0;
    private int lift6 = 0;     private int arm6 = 0;
    private int lift7 = 0;     private int arm7 = 0;
    
    @Override
    public void runOpMode() {
        stacker = new Stacker(this);
        stacker.unClampStone();

        waitForStart();

        while(opModeIsActive()){
            stacker.setDepositPower(gamepad1.left_stick_y);
            stacker.setLiftPower(gamepad1.right_stick_y);
            
            if(gamepad1.b){
                stacker.clampStone();
            }
            else{
                stacker.unClampStone();
            }
            
            telemetry.addData("Lift Pos", stacker.getLiftPosition());
            telemetry.addData("Arm Pos", stacker.getArmPosition());
            telemetry.update();
        }
    }
}