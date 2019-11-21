package org.firstinspires.ftc.teamcode.RobotClasses;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@SuppressWarnings("FieldCanBeLocal")
public class Stacker {

    //Electronics
    private DcMotorEx liftMotor;
    private DcMotorEx depositMotor;
    private Servo stoneClamp;
    
    private final double clampPos = 0.9;
    private final double unClampPos = 0.65;


    private final int armpos[] = {2160,1990,1700,1560,1260,1260,1260};
    private final int liftpos[] = {0,0,0,0,250,780,1250};

    private int currentStackHeight = 0;

    //OpMode Stuff
    private LinearOpMode op;
    private HardwareMap hardwareMap;

    public Stacker(LinearOpMode op){

        this.op = op;
        this.hardwareMap = op.hardwareMap;

        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        depositMotor = hardwareMap.get(DcMotorEx.class, "depositMotor");
        stoneClamp = hardwareMap.get(Servo.class, "stoneClamp");
        
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        depositMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    
        depositMotor.setTargetPosition(0);
        depositMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        liftMotor.setPower(0);
        depositMotor.setPower(0);
        
        op.telemetry.addData("Status", "Stacker Initialized");
        op.telemetry.update();
    }
    public void setLiftControls(double power, int ticks){
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(power);
        liftMotor.setTargetPosition(ticks);
    }
    public void setDepositControls(double power, int ticks){
        depositMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        depositMotor.setPower(power);
        depositMotor.setTargetPosition(ticks);
    }
    
    public void setDepositPower(double power){
        depositMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        depositMotor.setPower(power);
        
    }
    public void setLiftPower(double power){
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(power);
    }
    public void deposit(){
        setLiftControls(1, liftpos[currentStackHeight]);
        setDepositControls(0.75, armpos[currentStackHeight]);
    }


    public void nextLevel(){
        currentStackHeight++;
    }
    public void lastLevel(){
        currentStackHeight--;
    }
    
    public void clampStone() {
        stoneClamp.setPosition(clampPos);
    }

    public void unClampStone() {
        stoneClamp.setPosition(unClampPos);
    }
    
    public double getLiftPosition() {
        return liftMotor.getCurrentPosition();
    }
    public double getArmPosition() {
        return depositMotor.getCurrentPosition();
    }
}
