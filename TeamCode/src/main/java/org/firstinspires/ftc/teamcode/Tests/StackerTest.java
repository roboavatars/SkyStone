package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotClasses.Stacker;

@TeleOp(name="Stacker Test") @SuppressWarnings("FieldCanBeLocal")
public class StackerTest extends LinearOpMode {

    private Stacker stacker;
    

    
    @Override
    public void runOpMode() {
        stacker = new Stacker(this);
        stacker.unClampStone();

        waitForStart();

        while(opModeIsActive()){

            
            if(gamepad1.x){
                stacker.deposit();
            }

            
            telemetry.addData("Lift Pos", stacker.getLiftPosition());
            telemetry.addData("Arm Pos", stacker.getArmPosition());
            telemetry.update();
        }
    }
}